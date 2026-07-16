// Global Application State
// isSegmentEnd distinguishes lineTo (segment boundary ends) from addWaypoint (smoothed curves)
let waypoints = [
    { x: -50, y: -50, heading: 0, cornerRadius: 30, maxSpeed: 0.8, activeHeadingControl: false, linearHeadingInterpolation: false, positionTolerance: 1.0, stopVelocity: 10, isSegmentEnd: true },
    { x: -20, y: 30, heading: 90, cornerRadius: 30, maxSpeed: 1.0, activeHeadingControl: false, linearHeadingInterpolation: false, positionTolerance: 1.0, stopVelocity: 10, isSegmentEnd: false },
    { x: 40, y: -30, heading: 180, cornerRadius: 30, maxSpeed: 0.7, activeHeadingControl: false, linearHeadingInterpolation: false, positionTolerance: 1.5, stopVelocity: 5, isSegmentEnd: true },
    { x: 50, y: 50, heading: 270, cornerRadius: 30, maxSpeed: 0.9, activeHeadingControl: false, linearHeadingInterpolation: false, positionTolerance: 1.0, stopVelocity: 10, isSegmentEnd: true }
];

// Global Configuration Limits
const config = {
    maxVel: 90,
    maxAccel: 200,
    maxDecel: 60,
    tractionLimit: 110,
    startVel: 45,
    endVel: 0,
    driveEfficiency: 50,
    settlingDelay: 300 // ms of PID settling pause duration at segment boundaries
};

// Simulation State
let simulation = {
    isPlaying: false,
    progress: 0, 
    speed: 1.0,  
    animationFrameId: null,
    lastTime: null,
    currentTime: 0 
};

// Selected waypoint index
let selectedWpIndex = null;
let isDraggingWpIndex = null;
let isRotatingWpIndex = null;

// Trajectory data
let smoothPathData = [];
let totalPathLength = 0;
let estimatedTotalTime = 0;

// Coordinate mapping: center is (0,0), X forward (up), Y right (right)
function userToSvg(x, y) {
    return {
        x: y, // Screen X = User Y
        y: -x // Screen Y = -User X
    };
}

function svgToUser(svgX, svgY) {
    return {
        x: -svgY,
        y: svgX
    };
}

function normalizeAngleDeg(deg) {
    while (deg > 180) deg -= 360;
    while (deg < -180) deg += 360;
    return deg;
}

// DOM elements
const fieldSvg = document.getElementById('field-svg');
const trajectoryGroup = document.getElementById('trajectory-group');
const waypointsGroup = document.getElementById('waypoints-group');
const waypointList = document.getElementById('waypoint-list');
const robotShadow = document.getElementById('robot-shadow');
const robotLive = document.getElementById('robot-live');
const coordTooltip = document.getElementById('coord-tooltip');

const totalLengthVal = document.getElementById('total-length-val');
const totalTimeVal = document.getElementById('total-time-val');
const simStateVal = document.getElementById('simulation-state');

const playBtn = document.getElementById('play-btn');
const playIcon = document.getElementById('play-icon');
const pauseIcon = document.getElementById('pause-icon');
const timelineSlider = document.getElementById('timeline');
const speedSelect = document.getElementById('sim-speed-select');

const javaCodeIo = document.getElementById('java-code-io');
const importBtn = document.getElementById('import-btn');
const copyBtn = document.getElementById('copy-btn');
const clearBtn = document.getElementById('clear-btn');
const addWaypointBtn = document.getElementById('add-waypoint-btn');
const addLineToBtn = document.getElementById('add-lineto-btn');

// Initial Setup
window.addEventListener('DOMContentLoaded', () => {
    loadInputs();
    updateTrajectory();
    generateJavaCode();
    rebuildWaypointsUI();
    setupEventListeners();
});

function loadInputs() {
    config.maxVel = parseFloat(document.getElementById('maxVel').value) || 90;
    config.maxAccel = parseFloat(document.getElementById('maxAccel').value) || 200;
    config.maxDecel = parseFloat(document.getElementById('maxDecel').value) || 60;
    config.tractionLimit = parseFloat(document.getElementById('tractionLimit').value) || 110;
    config.startVel = parseFloat(document.getElementById('startVel').value) || 45;
    config.endVel = parseFloat(document.getElementById('endVel').value) || 0;
    config.driveEfficiency = parseFloat(document.getElementById('driveEfficiency').value) || 72;
}

function isStopNode(idx) {
    if (idx <= 0 || idx >= waypoints.length) return false;
    if (idx === waypoints.length - 1) return true;
    const nextWp = waypoints[idx + 1];
    return nextWp && nextWp.isSegmentEnd;
}

function getSegmentEndWpIndex(wpIndex) {
    let idx = wpIndex;
    while (idx < waypoints.length - 1 && !isStopNode(idx)) {
        idx++;
    }
    return idx;
}

// Smooth corner Bézier curve math matching Snap.java
function computeSmoothedPath() {
    if (waypoints.length === 0) return [];
    if (waypoints.length === 1) {
        return [{
            x: waypoints[0].x,
            y: waypoints[0].y,
            heading: waypoints[0].heading,
            maxSpeed: waypoints[0].maxSpeed,
            segmentIndex: 0,
            isSegmentEnd: true
        }];
    }

    // Resolve segment-level parameters to match SnapPathBuilder segment boundary commits
    const resolvedWaypoints = waypoints.map((wp, idx) => {
        const endWpIdx = getSegmentEndWpIndex(idx);
        const endWp = waypoints[endWpIdx];
        return {
            ...wp,
            maxSpeed: endWp.maxSpeed,
            positionTolerance: endWp.positionTolerance,
            stopVelocity: endWp.stopVelocity
            // activeHeadingControl and linearHeadingInterpolation removed from here so they stay native to wp
        };
    });

    let smoothPoints = [];

    // Setup first point
    smoothPoints.push({
        x: resolvedWaypoints[0].x,
        y: resolvedWaypoints[0].y,
        heading: resolvedWaypoints[0].heading,
        maxSpeed: resolvedWaypoints[0].maxSpeed,
        segmentIndex: 1,
        isSegmentEnd: false
    });

    // Helper to add straight line between points with heading interpolation
    function addStraightLine(startPt, endPt, segmentIndex) {
        const dist = Math.hypot(endPt.x - startPt.x, endPt.y - startPt.y);
        const steps = Math.max(1, Math.ceil(dist / 2.0));
        const startH = startPt.heading;
        const endH = endPt.heading;
        const linearHeading = endPt.linearHeadingInterpolation;

        for (let k = 1; k <= steps; k++) {
            const t = k / steps;
            const h = linearHeading ? normalizeAngleDeg(startH + t * normalizeAngleDeg(endH - startH)) : endH;
            smoothPoints.push({
                x: startPt.x + t * (endPt.x - startPt.x),
                y: startPt.y + t * (endPt.y - startPt.y),
                heading: h,
                maxSpeed: endPt.maxSpeed,
                segmentIndex: segmentIndex,
                isSegmentEnd: false
            });
        }
    }

    // Step through waypoints sequentially
    for (let i = 1; i < resolvedWaypoints.length; i++) {
        const prevWp = resolvedWaypoints[i - 1];
        const currWp = resolvedWaypoints[i];
        const nextWp = resolvedWaypoints[i + 1];

        if (isStopNode(i) || !nextWp) {
            // Straight line connection to currWp
            const lastPt = smoothPoints[smoothPoints.length - 1];
            addStraightLine(lastPt, currWp, i);
            smoothPoints[smoothPoints.length - 1].isSegmentEnd = true;
            continue;
        }

        // We can smooth the corner at currWp (since it is an intermediate addWaypoint)
        const p1 = prevWp;
        const p2 = currWp;
        const p3 = nextWp;

        const v1x = p1.x - p2.x;
        const v1y = p1.y - p2.y;
        const v2x = p3.x - p2.x;
        const v2y = p3.y - p2.y;

        const d1 = Math.hypot(v1x, v1y);
        const d2 = Math.hypot(v2x, v2y);

        if (d1 < 1e-6 || d2 < 1e-6) {
            const lastPt = smoothPoints[smoothPoints.length - 1];
            addStraightLine(lastPt, p2, i);
            continue;
        }

        const dotProduct = Math.max(-1.0, Math.min(1.0, (v1x * v2x + v1y * v2y) / (d1 * d2)));
        const angle = Math.acos(dotProduct);

        if (Math.abs(angle - Math.PI) < 1e-3 || angle < 1e-3) {
            const lastPt = smoothPoints[smoothPoints.length - 1];
            addStraightLine(lastPt, p2, i);
            continue;
        }

        const cornerRad = p2.cornerRadius || 30.0;
        const safeRadius = Math.min(cornerRad, Math.min(d1, d2) * 0.4 * Math.tan(angle / 2));
        const distToTangent = safeRadius / Math.tan(angle / 2);

        const startArcX = p2.x + (v1x / d1) * distToTangent;
        const startArcY = p2.y + (v1y / d1) * distToTangent;
        const lastPt = smoothPoints[smoothPoints.length - 1];
        
        // Add straight section to the start of the arc
        const distToStartArc = Math.hypot(startArcX - lastPt.x, startArcY - lastPt.y);
        if (distToStartArc > 0.5) {
            const steps = Math.max(1, Math.ceil(distToStartArc / 2.0));
            const startH = lastPt.heading;
            const endH = p2.heading;
            const linearHeading = p2.linearHeadingInterpolation;
            for (let k = 1; k <= steps; k++) {
                const t = k / steps;
                const h = linearHeading ? normalizeAngleDeg(startH + t * normalizeAngleDeg(endH - startH)) : endH;
                smoothPoints.push({
                    x: lastPt.x + t * (startArcX - lastPt.x),
                    y: lastPt.y + t * (startArcY - lastPt.y),
                    heading: h,
                    maxSpeed: p2.maxSpeed,
                    segmentIndex: i,
                    isSegmentEnd: false
                });
            }
        }

        const startArc = { x: startArcX, y: startArcY };
        const endArc = {
            x: p2.x + (v2x / d2) * distToTangent,
            y: p2.y + (v2y / d2) * distToTangent
        };

        // Add smoothed Bézier arc points
        const estimatedArcLength = Math.hypot(startArc.x - p2.x, startArc.y - p2.y) + Math.hypot(p2.x - endArc.x, p2.y - endArc.y);
        const N = Math.max(3, Math.ceil(estimatedArcLength / 2.0));

        for (let j = 0; j < N; j++) {
            const t = j / (N - 1);
            const mt = 1.0 - t;

            const bx = mt * mt * startArc.x + 2 * mt * t * p2.x + t * t * endArc.x;
            const by = mt * mt * startArc.y + 2 * mt * t * p2.y + t * t * endArc.y;
            const bh = normalizeAngleDeg(p2.heading + t * normalizeAngleDeg(p3.heading - p2.heading));

            smoothPoints.push({
                x: bx,
                y: by,
                heading: bh,
                maxSpeed: p2.maxSpeed,
                segmentIndex: i,
                isSegmentEnd: false
            });
        }
    }

    // Force final waypoint isSegmentEnd is true
    if (smoothPoints.length > 0) {
        smoothPoints[smoothPoints.length - 1].isSegmentEnd = true;
    }

    // Compute dynamic headings matching activeHeadingControl, linear interpolations, and distance snaps
    applyHeadingControl(smoothPoints);

    return smoothPoints;
}

// Helper to check if a waypoint is the end of a consecutive active heading control chain
function isActiveHeadingChainEnd(wpIdx) {
    if (wpIdx < 0 || wpIdx >= waypoints.length) return true;
    const wp = waypoints[wpIdx];
    if (!wp.activeHeadingControl) return false;

    // If it's the last waypoint, it's the end of the chain
    if (wpIdx === waypoints.length - 1) return true;

    // Check if the immediate next waypoint continues the chain
    const nextWp = waypoints[wpIdx + 1];
    if (nextWp && nextWp.activeHeadingControl) {
        return false;
    }
    return true;
}

// Compute the realistic headings matching Snap.java dynamics
function applyHeadingControl(smoothPoints) {
    if (smoothPoints.length === 0) return;

    const N = smoothPoints.length;
    let distToSegmentEnd = 0;

    // Pass 1: Compute distToSegmentEnd for each point
    for (let i = N - 1; i >= 0; i--) {
        if (smoothPoints[i].isSegmentEnd) {
            distToSegmentEnd = 0;
        } else if (i < N - 1) {
            const ds = Math.hypot(smoothPoints[i+1].x - smoothPoints[i].x, smoothPoints[i+1].y - smoothPoints[i].y);
            distToSegmentEnd += ds;
        }
        smoothPoints[i].distToSegmentEnd = distToSegmentEnd;
    }

    let prevHeading = waypoints[0].heading;

    // Pass 2: Apply active heading control direction of travel
    for (let k = 0; k < N; k++) {
        const pt = smoothPoints[k];
        const segIdx = pt.segmentIndex;
        const targetWp = waypoints[segIdx]; // The waypoint ending this specific sub-segment

        if (!targetWp) {
            pt.heading = prevHeading;
            continue;
        }

        const activeHeading = targetWp.activeHeadingControl;
        const isChainEnd = isActiveHeadingChainEnd(segIdx);

        if (activeHeading && (!isChainEnd || pt.distToSegmentEnd >= 5.0)) {
            // Active heading control: align with direction of travel using robust lookahead
            let forward = prevHeading;
            let found = false;
            // Scan forward to find a point that is at least 3 inches away
            for (let j = k + 1; j < N; j++) {
                const dx = smoothPoints[j].x - pt.x;
                const dy = smoothPoints[j].y - pt.y;
                const dist = Math.hypot(dx, dy);
                if (dist > 3.0) {
                    forward = Math.atan2(dy, dx) * 180 / Math.PI;
                    found = true;
                    break;
                }
            }
            // If near the end of the path and didn't find a point 3 inches ahead, scan backward
            if (!found) {
                for (let j = k - 1; j >= 0; j--) {
                    const dx = pt.x - smoothPoints[j].x;
                    const dy = pt.y - smoothPoints[j].y;
                    const dist = Math.hypot(dx, dy);
                    if (dist > 3.0) {
                        forward = Math.atan2(dy, dx) * 180 / Math.PI;
                        found = true;
                        break;
                    }
                }
            }

            const backward = forward + 180;
            const forwardCost = Math.abs(normalizeAngleDeg(forward - prevHeading));
            const backwardCost = Math.abs(normalizeAngleDeg(backward - prevHeading));
            const targetHeading = (forwardCost <= backwardCost) ? forward : backward;

            pt.heading = normalizeAngleDeg(targetHeading);
        } else {
            // Use pre-computed path heading (either constant or interpolated)
            pt.heading = normalizeAngleDeg(pt.heading);
        }
        prevHeading = pt.heading;
    }
}

// Generate velocity profiles matching curvature limits, traction limits, and drive efficiency
function generateVelocityProfile(smoothPoints) {
    if (smoothPoints.length === 0) return [];
    
    const N = smoothPoints.length;
    let vRaw = new Array(N).fill(0);
    
    vRaw[0] = config.maxVel * (smoothPoints[0].maxSpeed || 1.0);
    vRaw[N - 1] = config.maxVel * (smoothPoints[N - 1].maxSpeed || 1.0);

    for (let i = 1; i < N - 1; i++) {
        const p1 = smoothPoints[i - 1];
        const p2 = smoothPoints[i];
        const p3 = smoothPoints[i + 1];

        const d12 = Math.hypot(p2.x - p1.x, p2.y - p1.y);
        const d23 = Math.hypot(p3.x - p2.x, p3.y - p2.y);
        const d13 = Math.hypot(p3.x - p1.x, p3.y - p1.y);

        const area = 0.5 * Math.abs(p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));

        let curvature = 0.0;
        const denom = d12 * d23 * d13;
        if (denom > 1e-6 && area > 1e-6) {
            curvature = (4.0 * area) / denom;
        }

        let vCurve = Infinity;
        if (curvature > 1e-6) {
            vCurve = Math.sqrt(config.tractionLimit / curvature);
        }
        vRaw[i] = Math.min(config.maxVel * (p2.maxSpeed || 1.0), vCurve);
    }

    let speeds = [...vRaw];
    speeds[N - 1] = Math.min(speeds[N - 1], config.endVel);
    speeds[0] = Math.min(speeds[0], config.startVel);

    // Enforce end velocity at all segment boundaries (lineTo waypoints)
    for (let i = 0; i < N; i++) {
        if (smoothPoints[i].isSegmentEnd) {
            speeds[i] = Math.min(speeds[i], config.endVel);
        }
    }

    // Deceleration backward pass
    for (let i = N - 2; i >= 0; i--) {
        const ds = Math.hypot(smoothPoints[i + 1].x - smoothPoints[i].x, smoothPoints[i + 1].y - smoothPoints[i].y);
        const vDecel = Math.sqrt(speeds[i + 1] * speeds[i + 1] + 2.0 * config.maxDecel * ds);
        speeds[i] = Math.min(speeds[i], vDecel);
    }

    // Acceleration forward pass
    for (let i = 1; i < N; i++) {
        const ds = Math.hypot(smoothPoints[i].x - smoothPoints[i - 1].x, smoothPoints[i].y - smoothPoints[i - 1].y);
        const vAccel = Math.sqrt(speeds[i - 1] * speeds[i - 1] + 2.0 * config.maxAccel * ds);
        speeds[i] = Math.min(speeds[i], vAccel);
    }

    let integratedData = [];
    let runningDist = 0;
    let runningTime = 0;

    const timeScaleFactor = 100.0 / Math.max(40, config.driveEfficiency);

    for (let i = 0; i < N; i++) {
        let ds = 0;
        if (i > 0) {
            ds = Math.hypot(smoothPoints[i].x - smoothPoints[i - 1].x, smoothPoints[i].y - smoothPoints[i - 1].y);
            runningDist += ds;
            const avgSpeed = Math.max(0.1, (speeds[i] + speeds[i - 1]) / 2.0);
            
            runningTime += (ds / avgSpeed) * timeScaleFactor;
        }

        // CRITICAL SETTLING overhead delay at segment boundaries
        const pauseDur = (smoothPoints[i].isSegmentEnd && i > 0) ? (config.settlingDelay / 1000.0) : 0.0;

        integratedData.push({
            ...smoothPoints[i],
            targetSpeed: speeds[i],
            distance: runningDist,
            time: runningTime,
            pauseDuration: pauseDur
        });

        // The next segment starts after this pause completes
        runningTime += pauseDur;
    }

    return integratedData;
}


// Recalculates paths (DOES NOT generate Java code to avoid dragging lag)
function updateTrajectory() {
    loadInputs();
    const rawSmooth = computeSmoothedPath();
    smoothPathData = generateVelocityProfile(rawSmooth);
    
    if (smoothPathData.length > 0) {
        totalPathLength = smoothPathData[smoothPathData.length - 1].distance;
        estimatedTotalTime = smoothPathData[smoothPathData.length - 1].time;
    } else {
        totalPathLength = 0;
        estimatedTotalTime = 0;
    }

    totalLengthVal.textContent = `${totalPathLength.toFixed(1)}"`;
    totalTimeVal.textContent = `${estimatedTotalTime.toFixed(2)}s`;

    renderPathLine();
    
    if (simulation.isPlaying || simulation.progress > 0) {
        if (simulation.progress >= estimatedTotalTime) {
            simulation.progress = 0;
            simulation.isPlaying = false;
        }
        renderRobotSimulationFrame();
    }
}

// Render the smoothed colored path based on velocity
function renderPathLine() {
    trajectoryGroup.innerHTML = '';
    if (smoothPathData.length < 2) return;

    for (let i = 1; i < smoothPathData.length; i++) {
        const pt1 = smoothPathData[i - 1];
        const pt2 = smoothPathData[i];

        const s1 = userToSvg(pt1.x, pt1.y);
        const s2 = userToSvg(pt2.x, pt2.y);

        const normSpeed = Math.min(1.0, pt2.targetSpeed / config.maxVel);
        const hue = normSpeed * 130; 
        const strokeColor = `hsl(${hue}, 85%, 50%)`;

        const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
        line.setAttribute('x1', s1.x);
        line.setAttribute('y1', s1.y);
        line.setAttribute('x2', s2.x);
        line.setAttribute('y2', s2.y);
        line.setAttribute('stroke', strokeColor);
        line.setAttribute('stroke-width', '0.6');
        line.setAttribute('class', 'path-smooth-line');
        trajectoryGroup.appendChild(line);
    }

    for (let i = 1; i < waypoints.length; i++) {
        const s1 = userToSvg(waypoints[i - 1].x, waypoints[i - 1].y);
        const s2 = userToSvg(waypoints[i].x, waypoints[i].y);

        const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
        line.setAttribute('x1', s1.x);
        line.setAttribute('y1', s1.y);
        line.setAttribute('x2', s2.x);
        line.setAttribute('y2', s2.y);
        line.setAttribute('class', 'path-raw-line');
        trajectoryGroup.appendChild(line);
    }
}

// Rebuild lists and markers DOM structure (only on items adds, deletes, or code imports)
function rebuildWaypointsUI() {
    waypointsGroup.innerHTML = '';
    waypointList.innerHTML = '';

    waypoints.forEach((wp, idx) => {
        const svgCoords = userToSvg(wp.x, wp.y);

        const wpContainer = document.createElementNS('http://www.w3.org/2000/svg', 'g');
        wpContainer.setAttribute('class', `wp-node`);
        wpContainer.setAttribute('id', `wp-group-${idx}`);
        if (idx === selectedWpIndex) {
            wpContainer.classList.add('active');
        }

        const headRad = (wp.heading - 90) * Math.PI / 180;
        const headLength = 7.0;
        const hLine = document.createElementNS('http://www.w3.org/2000/svg', 'line');
        hLine.setAttribute('x1', svgCoords.x);
        hLine.setAttribute('y1', svgCoords.y);
        hLine.setAttribute('x2', svgCoords.x + headLength * Math.cos(headRad));
        hLine.setAttribute('y2', svgCoords.y + headLength * Math.sin(headRad));
        hLine.setAttribute('class', 'wp-heading-line');
        hLine.setAttribute('id', `wp-line-${idx}`);
        wpContainer.appendChild(hLine);

        const hHandle = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
        hHandle.setAttribute('cx', svgCoords.x + headLength * Math.cos(headRad));
        hHandle.setAttribute('cy', svgCoords.y + headLength * Math.sin(headRad));
        hHandle.setAttribute('r', '1.2');
        hHandle.setAttribute('class', 'wp-heading-handle');
        hHandle.setAttribute('id', `wp-handle-${idx}`);
        hHandle.addEventListener('mousedown', (e) => {
            e.stopPropagation();
            isRotatingWpIndex = idx;
            selectedWpIndex = idx;
            highlightActiveElements();
        });
        wpContainer.appendChild(hHandle);

        const circle = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
        circle.setAttribute('cx', svgCoords.x);
        circle.setAttribute('cy', svgCoords.y);
        circle.setAttribute('r', '2.0');
        circle.setAttribute('class', 'wp-anchor');
        circle.setAttribute('id', `wp-anchor-${idx}`);
        if (idx === selectedWpIndex) {
            circle.setAttribute('filter', 'url(#glow-cyan)');
        }
        
        circle.addEventListener('mousedown', (e) => {
            e.stopPropagation();
            selectedWpIndex = idx;
            isDraggingWpIndex = idx;
            highlightActiveElements();
        });

        wpContainer.appendChild(circle);

        const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
        label.setAttribute('x', svgCoords.x);
        label.setAttribute('y', svgCoords.y - 3.5);
        label.setAttribute('class', 'wp-label');
        label.setAttribute('id', `wp-label-${idx}`);
        label.textContent = idx;
        wpContainer.appendChild(label);

        waypointsGroup.appendChild(wpContainer);

        const wpItem = document.createElement('div');
        wpItem.className = `waypoint-item`;
        wpItem.setAttribute('id', `sidebar-item-${idx}`);
        if (idx === selectedWpIndex) {
            wpItem.classList.add('active');
        }
        
        wpItem.addEventListener('click', () => {
            selectedWpIndex = idx;
            highlightActiveElements();
        });

        // 2-column layout. Heading spans 2 columns at waypoint 0.
        const stopNode = isStopNode(idx);
        const isStart = idx === 0;
        
        // Find the stop node that resolves parameters for this waypoint
        const endWpIdx = getSegmentEndWpIndex(idx);
        const endWp = waypoints[endWpIdx];

        // Determine value and state of each field
        const maxSpeedVal = stopNode ? (wp.maxSpeed !== null ? wp.maxSpeed : 1.0) : (endWp ? endWp.maxSpeed : 1.0);
        const posTolVal = stopNode ? (wp.positionTolerance || 1.0) : (endWp ? endWp.positionTolerance : 1.0);
        const activeHeadingChecked = wp.activeHeadingControl || false;
        const linearHeadingChecked = wp.linearHeadingInterpolation || false;

        wpItem.innerHTML = `
            <div class="waypoint-header">
                <span class="waypoint-title">Waypoint ${idx} ${isStart ? '(Start)' : ''} ${wp.isSegmentEnd && idx > 0 ? '[lineTo]' : ''} ${stopNode && idx > 0 ? '[stop]' : ''}</span>
                <div class="waypoint-actions">
                    <button class="btn-remove" title="Delete Waypoint">&times;</button>
                </div>
            </div>
            <div class="waypoint-fields">
                <div class="input-group">
                    <label>X (in)</label>
                    <input type="number" class="wp-field-x" id="input-x-${idx}" value="${wp.x.toFixed(1)}" step="0.5">
                </div>
                <div class="input-group">
                    <label>Y (in)</label>
                    <input type="number" class="wp-field-y" id="input-y-${idx}" value="${wp.y.toFixed(1)}" step="0.5">
                </div>
                <div class="input-group" ${isStart ? 'style="grid-column: span 2;"' : ''}>
                    <label>Heading (°)</label>
                    <input type="number" class="wp-field-heading" id="input-heading-${idx}" value="${Math.round(wp.heading)}" step="5">
                </div>
                ${!isStart ? `
                <div class="input-group">
                    <label>Corner Rad</label>
                    <input type="number" class="wp-field-rad" id="input-rad-${idx}" value="${wp.cornerRadius || 30}" step="5" ${stopNode ? 'disabled style="opacity:0.4;"' : ''}>
                </div>
                <div class="input-group">
                    <label>Max Speed</label>
                    <input type="number" class="wp-field-speed" id="input-speed-${idx}" value="${maxSpeedVal.toFixed(2)}" step="0.05" min="0.1" max="1.0" ${!stopNode ? 'disabled style="opacity:0.6;"' : ''}>
                </div>
                <div class="input-group" style="grid-column: span 2;">
                    <label>Pos Tol (in)</label>
                    <input type="number" class="wp-field-tol" id="input-tol-${idx}" value="${posTolVal.toFixed(1)}" step="0.1" ${!stopNode ? 'disabled style="opacity:0.6;"' : ''}>
                </div>
                ` : ''}
            </div>
            ${!isStart ? `
            <div class="waypoint-toggles">
                <label class="toggle-group">
                    <input type="checkbox" class="wp-toggle-active-heading" id="input-active-heading-${idx}" ${activeHeadingChecked ? 'checked' : ''}>
                    Active Heading Control
                </label>
                <label class="toggle-group">
                    <input type="checkbox" class="wp-toggle-linear-heading" id="input-linear-heading-${idx}" ${linearHeadingChecked ? 'checked' : ''}>
                    Linear Heading Interpolation
                </label>
            </div>
            ` : ''}
        `;

        wpItem.querySelector('.wp-field-x').addEventListener('input', (e) => {
            wp.x = parseFloat(e.target.value) || 0;
            updateWaypointSvgPosition(idx);
            updateTrajectory();
            generateJavaCode();
        });
        wpItem.querySelector('.wp-field-y').addEventListener('input', (e) => {
            wp.y = parseFloat(e.target.value) || 0;
            updateWaypointSvgPosition(idx);
            updateTrajectory();
            generateJavaCode();
        });
        wpItem.querySelector('.wp-field-heading').addEventListener('input', (e) => {
            wp.heading = normalizeAngleDeg(parseFloat(e.target.value) || 0);
            updateWaypointSvgPosition(idx);
            updateTrajectory();
            generateJavaCode();
        });

        if (idx > 0) {
            const radInput = wpItem.querySelector('.wp-field-rad');
            if (radInput) {
                radInput.addEventListener('input', (e) => {
                    wp.cornerRadius = Math.max(0, parseFloat(e.target.value) || 0);
                    updateTrajectory();
                    generateJavaCode();
                });
            }

            const speedInput = wpItem.querySelector('.wp-field-speed');
            if (speedInput) {
                speedInput.addEventListener('input', (e) => {
                    wp.maxSpeed = Math.max(0.1, Math.min(1.0, parseFloat(e.target.value) || 1.0));
                    const segEndIdx = getSegmentEndWpIndex(idx);
                    for (let k = 0; k < waypoints.length; k++) {
                        if (getSegmentEndWpIndex(k) === segEndIdx) {
                            waypoints[k].maxSpeed = wp.maxSpeed;
                            const otherInput = document.getElementById(`input-speed-${k}`);
                            if (otherInput && k !== idx) {
                                otherInput.value = wp.maxSpeed.toFixed(2);
                            }
                        }
                    }
                    updateTrajectory();
                    generateJavaCode();
                });
            }

            const tolInput = wpItem.querySelector('.wp-field-tol');
            if (tolInput) {
                tolInput.addEventListener('input', (e) => {
                    wp.positionTolerance = Math.max(0.1, parseFloat(e.target.value) || 1.0);
                    const segEndIdx = getSegmentEndWpIndex(idx);
                    for (let k = 0; k < waypoints.length; k++) {
                        if (getSegmentEndWpIndex(k) === segEndIdx) {
                            waypoints[k].positionTolerance = wp.positionTolerance;
                            const otherInput = document.getElementById(`input-tol-${k}`);
                            if (otherInput && k !== idx) {
                                otherInput.value = wp.positionTolerance.toFixed(1);
                            }
                        }
                    }
                    updateTrajectory();
                    generateJavaCode();
                });
            }

            const ahToggle = wpItem.querySelector('.wp-toggle-active-heading');
            if (ahToggle) {
                ahToggle.addEventListener('change', (e) => {
                    wp.activeHeadingControl = e.target.checked;
                    updateTrajectory();
                    generateJavaCode();
                });
            }

            const lhToggle = wpItem.querySelector('.wp-toggle-linear-heading');
            if (lhToggle) {
                lhToggle.addEventListener('change', (e) => {
                    wp.linearHeadingInterpolation = e.target.checked;
                    updateTrajectory();
                    generateJavaCode();
                });
            }
        }

        wpItem.querySelector('.btn-remove').addEventListener('click', (e) => {
            e.stopPropagation();
            waypoints.splice(idx, 1);
            if (selectedWpIndex === idx) selectedWpIndex = null;
            
            // Adjust segment boundary ends
            if (waypoints.length > 0) {
                waypoints[waypoints.length - 1].isSegmentEnd = true;
            }

            updateTrajectory();
            generateJavaCode();
            rebuildWaypointsUI();
        });

        waypointList.appendChild(wpItem);
    });
}

function updateWaypointSvgPosition(idx) {
    const wp = waypoints[idx];
    const coords = userToSvg(wp.x, wp.y);

    const circle = document.getElementById(`wp-anchor-${idx}`);
    if (circle) {
        circle.setAttribute('cx', coords.x);
        circle.setAttribute('cy', coords.y);
    }

    const label = document.getElementById(`wp-label-${idx}`);
    if (label) {
        label.setAttribute('x', coords.x);
        label.setAttribute('y', coords.y - 3.5);
    }

    const headRad = (wp.heading - 90) * Math.PI / 180;
    const headLength = 7.0;
    const hLine = document.getElementById(`wp-line-${idx}`);
    if (hLine) {
        hLine.setAttribute('x1', coords.x);
        hLine.setAttribute('y1', coords.y);
        hLine.setAttribute('x2', coords.x + headLength * Math.cos(headRad));
        hLine.setAttribute('y2', coords.y + headLength * Math.sin(headRad));
    }

    const hHandle = document.getElementById(`wp-handle-${idx}`);
    if (hHandle) {
        hHandle.setAttribute('cx', coords.x + headLength * Math.cos(headRad));
        hHandle.setAttribute('cy', coords.y + headLength * Math.sin(headRad));
    }

    const inputX = document.getElementById(`input-x-${idx}`);
    const inputY = document.getElementById(`input-y-${idx}`);
    const inputH = document.getElementById(`input-heading-${idx}`);

    if (inputX && document.activeElement !== inputX) inputX.value = wp.x.toFixed(1);
    if (inputY && document.activeElement !== inputY) inputY.value = wp.y.toFixed(1);
    if (inputH && document.activeElement !== inputH) inputH.value = Math.round(wp.heading);
}

function highlightActiveElements() {
    for (let i = 0; i < waypoints.length; i++) {
        const wpGrp = document.getElementById(`wp-group-${i}`);
        const sbItem = document.getElementById(`sidebar-item-${i}`);
        const circle = document.getElementById(`wp-anchor-${i}`);

        if (i === selectedWpIndex) {
            if (wpGrp) wpGrp.classList.add('active');
            if (sbItem) sbItem.classList.add('active');
            if (circle) circle.setAttribute('filter', 'url(#glow-cyan)');
        } else {
            if (wpGrp) wpGrp.classList.remove('active');
            if (sbItem) sbItem.classList.remove('active');
            if (circle) circle.removeAttribute('filter');
        }
    }
    
    if (simulation.progress > 0 || simulation.isPlaying) {
        renderRobotSimulationFrame();
    }
}

// Mouse events and dragging handling
function setupEventListeners() {
    window.addEventListener('mousemove', (e) => {
        const rect = fieldSvg.getBoundingClientRect();
        const svgX = ((e.clientX - rect.left) / rect.width) * 200 - 100;
        const svgY = ((e.clientY - rect.top) / rect.height) * 200 - 100;
        const fieldCoords = svgToUser(svgX, svgY);

        const isOverField = (e.clientX >= rect.left && e.clientX <= rect.right &&
                             e.clientY >= rect.top && e.clientY <= rect.bottom);
        if (isOverField) {
            coordTooltip.textContent = `X: ${fieldCoords.x.toFixed(1)}", Y: ${fieldCoords.y.toFixed(1)}"`;
            coordTooltip.style.opacity = '1';
            coordTooltip.style.left = `${e.clientX + 15}px`;
            coordTooltip.style.top = `${e.clientY + 15}px`;
        } else {
            coordTooltip.style.opacity = '0';
        }

        if (isDraggingWpIndex !== null) {
            const clampedX = Math.max(-96, Math.min(96, fieldCoords.x));
            const clampedY = Math.max(-96, Math.min(96, fieldCoords.y));
            
            waypoints[isDraggingWpIndex].x = Math.round(clampedX * 10) / 10;
            waypoints[isDraggingWpIndex].y = Math.round(clampedY * 10) / 10;
            
            updateWaypointSvgPosition(isDraggingWpIndex);
            updateTrajectory();
        } else if (isRotatingWpIndex !== null) {
            const wpCoords = userToSvg(waypoints[isRotatingWpIndex].x, waypoints[isRotatingWpIndex].y);
            const dx = svgX - wpCoords.x;
            const dy = svgY - wpCoords.y;
            const rad = Math.atan2(dy, dx);
            let angle = rad * 180 / Math.PI + 90;
            
            waypoints[isRotatingWpIndex].heading = Math.round(normalizeAngleDeg(angle));
            
            updateWaypointSvgPosition(isRotatingWpIndex);
            updateTrajectory();
        }
    });

    fieldSvg.addEventListener('mouseleave', () => {
        coordTooltip.style.opacity = '0';
    });

    window.addEventListener('mouseup', () => {
        if (isDraggingWpIndex !== null || isRotatingWpIndex !== null) {
            generateJavaCode();
        }
        isDraggingWpIndex = null;
        isRotatingWpIndex = null;
    });

    // Double click to add waypoint
    fieldSvg.addEventListener('dblclick', (e) => {
        const rect = fieldSvg.getBoundingClientRect();
        const svgX = ((e.clientX - rect.left) / rect.width) * 200 - 100;
        const svgY = ((e.clientY - rect.top) / rect.height) * 200 - 100;
        const fieldCoords = svgToUser(svgX, svgY);

        const newWp = {
            x: Math.round(fieldCoords.x * 2) / 2,
            y: Math.round(fieldCoords.y * 2) / 2,
            heading: 0,
            cornerRadius: 30,
            maxSpeed: 1.0,
            activeHeadingControl: false,
            linearHeadingInterpolation: false,
            positionTolerance: 1.0,
            stopVelocity: 10,
            isSegmentEnd: false
        };

        if (waypoints.length > 0) {
            waypoints[waypoints.length - 1].isSegmentEnd = false; 
        }
        
        newWp.isSegmentEnd = true; 

        waypoints.push(newWp);
        selectedWpIndex = waypoints.length - 1;
        updateTrajectory();
        generateJavaCode();
        rebuildWaypointsUI();
    });

    // Constraints inputs changes
    document.querySelectorAll('.config-card input').forEach(input => {
        input.addEventListener('input', () => {
            updateTrajectory();
            generateJavaCode();
        });
    });

    // Simulation controls
    playBtn.addEventListener('click', toggleSimulation);
    timelineSlider.addEventListener('input', (e) => {
        simulation.progress = parseFloat(e.target.value);
        renderRobotSimulationFrame();
    });
    speedSelect.addEventListener('change', (e) => {
        simulation.speed = parseFloat(e.target.value);
    });

    // Import / Export Buttons
    importBtn.addEventListener('click', importJavaCode);
    copyBtn.addEventListener('click', () => {
        javaCodeIo.select();
        document.execCommand('copy');
        copyBtn.textContent = 'Copied!';
        setTimeout(() => copyBtn.textContent = 'Copy Code', 1500);
    });
    clearBtn.addEventListener('click', () => {
        waypoints = [];
        selectedWpIndex = null;
        updateTrajectory();
        generateJavaCode();
        rebuildWaypointsUI();
    });

    function addPoint(isLineTo) {
        if (waypoints.length === 0) {
            waypoints.push({
                x: 0,
                y: 0,
                heading: 0,
                cornerRadius: 30,
                maxSpeed: 1.0,
                activeHeadingControl: false,
                linearHeadingInterpolation: false,
                positionTolerance: 1.0,
                stopVelocity: 10,
                isSegmentEnd: false
            });
        }
        const lastWp = waypoints[waypoints.length - 1];
        const newWp = {
            x: Math.max(-94, Math.min(94, Math.round(lastWp.x + 15))),
            y: Math.max(-94, Math.min(94, Math.round(lastWp.y + 15))),
            heading: lastWp.heading,
            cornerRadius: 30,
            maxSpeed: 1.0,
            activeHeadingControl: false,
            linearHeadingInterpolation: false,
            positionTolerance: 1.0,
            stopVelocity: 10,
            isSegmentEnd: isLineTo
        };
        waypoints.push(newWp);
        selectedWpIndex = waypoints.length - 1;
        updateTrajectory();
        generateJavaCode();
        rebuildWaypointsUI();
    }

    addWaypointBtn.addEventListener('click', () => addPoint(false));
    addLineToBtn.addEventListener('click', () => addPoint(true));
}

// Generate Java code matching SnapPathBuilder fluent chain
function generateJavaCode() {
    if (waypoints.length === 0) {
        javaCodeIo.value = '';
        return;
    }

    let code = `PathChain path = new SnapPathBuilder(new Pose2D(${waypoints[0].x.toFixed(1)}, ${waypoints[0].y.toFixed(1)}, Math.toRadians(${Math.round(waypoints[0].heading)})))\n`;

    for (let i = 1; i < waypoints.length; i++) {
        const wp = waypoints[i];
        code += `    .setMaxSpeed(${wp.maxSpeed !== null ? wp.maxSpeed.toFixed(2) : '1.0'})\n`;
        if (!wp.isSegmentEnd) {
            code += `    .setCornerRadius(${wp.cornerRadius !== null ? wp.cornerRadius.toFixed(1) : '30.0'})\n`;
        }
        code += `    .setActiveHeadingControl(${wp.activeHeadingControl})\n`;
        code += `    .setLinearHeadingInterpolation(${wp.linearHeadingInterpolation})\n`;
        code += `    .setPositionTolerance(${wp.positionTolerance !== null ? wp.positionTolerance.toFixed(1) : '1.0'})\n`;
        
        if (wp.isSegmentEnd) {
            code += `    .lineTo(new Pose2D(${wp.x.toFixed(1)}, ${wp.y.toFixed(1)}, Math.toRadians(${Math.round(wp.heading)})))\n`;
        } else {
            code += `    .addWaypoint(new Pose2D(${wp.x.toFixed(1)}, ${wp.y.toFixed(1)}, Math.toRadians(${Math.round(wp.heading)})))\n`;
        }
    }
    code += `    .build();`;

    javaCodeIo.value = code;
}

// Robust tokenizing parser using a state-machine parentheses matcher
// Robust tokenizing parser using a state-machine parentheses matcher
function importJavaCode() {
    const code = javaCodeIo.value;
    if (!code.trim()) return;

    try {
        const parsed = parseFluentChains(code);
        const cleanCode = parsed.cleanCode;
        const methods = parsed.methods;

        let startPose = null;
        const startIdx = cleanCode.indexOf('new SnapPathBuilder');
        if (startIdx !== -1) {
            let i = startIdx + 'new SnapPathBuilder'.length;
            while (i < cleanCode.length && cleanCode[i] !== '(') {
                i++;
            }
            if (cleanCode[i] === '(') {
                i++;
                let startPos = i;
                let level = 1;
                while (i < cleanCode.length && level > 0) {
                    if (cleanCode[i] === '(') level++;
                    else if (cleanCode[i] === ')') level--;
                    i++;
                }
                if (level === 0) {
                    let args = cleanCode.substring(startPos, i - 1);
                    const poseRegex = /new\s+Pose2D\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*([\s\S]*)\)/i;
                    const poseMatch = args.match(poseRegex);
                    if (poseMatch) {
                        startPose = {
                            x: parseFloat(poseMatch[1]),
                            y: parseFloat(poseMatch[2]),
                            heading: parseHeadingToken(poseMatch[3])
                        };
                    }
                }
            }
        }

        if (!startPose) {
            alert("Error parsing Java: Could not find valid SnapPathBuilder starting Pose2D.");
            return;
        }

        const newWaypoints = [];
        newWaypoints.push({
            x: startPose.x,
            y: startPose.y,
            heading: normalizeAngleDeg(startPose.heading),
            cornerRadius: null,
            maxSpeed: null,
            activeHeadingControl: null,
            linearHeadingInterpolation: null,
            positionTolerance: null,
            stopVelocity: null,
            isSegmentEnd: true 
        });

        let chainConfig = {
            maxSpeed: null,
            cornerRadius: null,
            activeHeadingControl: null,
            linearHeadingInterpolation: null,
            positionTolerance: null,
            stopVelocity: null
        };



        methods.forEach(method => {
            const name = method.name;
            const args = method.args.trim();

            if (name === 'lineTo' || name === 'addWaypoint') {
                const poseRegex = /new\s+Pose2D\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*([\s\S]*)\)/i;
                const poseMatch = args.match(poseRegex);
                if (poseMatch) {
                    const xVal = parseFloat(poseMatch[1]);
                    const yVal = parseFloat(poseMatch[2]);
                    const hVal = parseHeadingToken(poseMatch[3]);

                    newWaypoints.push({
                        x: xVal,
                        y: yVal,
                        heading: normalizeAngleDeg(hVal),
                        cornerRadius: null,
                        maxSpeed: null,
                        activeHeadingControl: null,
                        linearHeadingInterpolation: null,
                        positionTolerance: null,
                        stopVelocity: null,
                        isSegmentEnd: (name === 'lineTo') 
                    });
                }
            } else if (name.startsWith('setChain')) {
                const param = name.substring(8); // e.g. "ActiveHeadingControl"
                if (param === 'MaxSpeed') chainConfig.maxSpeed = parseFloat(args);
                else if (param === 'CornerRadius') chainConfig.cornerRadius = parseFloat(args);
                else if (param === 'ActiveHeadingControl') chainConfig.activeHeadingControl = (args === 'true');
                else if (param === 'LinearHeadingInterpolation') chainConfig.linearHeadingInterpolation = (args === 'true');
                else if (param === 'PositionTolerance') chainConfig.positionTolerance = parseFloat(args);
                else if (param === 'StopVelocity') chainConfig.stopVelocity = parseFloat(args);
            } else if (newWaypoints.length > 0) {
                const lastWp = newWaypoints[newWaypoints.length - 1];
                if (name === 'setMaxSpeed') {
                    lastWp.maxSpeed = parseFloat(args);
                } else if (name === 'setCornerRadius') {
                    lastWp.cornerRadius = parseFloat(args);
                } else if (name === 'setActiveHeadingControl') {
                    lastWp.activeHeadingControl = (args === 'true');
                } else if (name === 'setLinearHeadingInterpolation') {
                    lastWp.linearHeadingInterpolation = (args === 'true');
                } else if (name === 'setPositionTolerance') {
                    lastWp.positionTolerance = parseFloat(args);
                } else if (name === 'setStopVelocity') {
                    lastWp.stopVelocity = parseFloat(args);
                }
            }
        });


        
        newWaypoints.forEach(wp => {
            wp.maxSpeed = (wp.maxSpeed !== null && !isNaN(wp.maxSpeed)) ? wp.maxSpeed :
                          (chainConfig.maxSpeed !== null && !isNaN(chainConfig.maxSpeed)) ? chainConfig.maxSpeed : 1.0;

            wp.cornerRadius = (wp.cornerRadius !== null && !isNaN(wp.cornerRadius)) ? wp.cornerRadius :
                              (chainConfig.cornerRadius !== null && !isNaN(chainConfig.cornerRadius)) ? chainConfig.cornerRadius : 30.0;

            wp.activeHeadingControl = (wp.activeHeadingControl !== null) ? wp.activeHeadingControl :
                                      (chainConfig.activeHeadingControl !== null) ? chainConfig.activeHeadingControl : false;

            wp.linearHeadingInterpolation = (wp.linearHeadingInterpolation !== null) ? wp.linearHeadingInterpolation :
                                            (chainConfig.linearHeadingInterpolation !== null) ? chainConfig.linearHeadingInterpolation : false;

            wp.positionTolerance = (wp.positionTolerance !== null && !isNaN(wp.positionTolerance)) ? wp.positionTolerance :
                                   (chainConfig.positionTolerance !== null && !isNaN(chainConfig.positionTolerance)) ? chainConfig.positionTolerance : 1.0;

            wp.stopVelocity = (wp.stopVelocity !== null && !isNaN(wp.stopVelocity)) ? wp.stopVelocity :
                              (chainConfig.stopVelocity !== null && !isNaN(chainConfig.stopVelocity)) ? chainConfig.stopVelocity : 10.0;
        });

        if (newWaypoints.length > 0) {
            newWaypoints[newWaypoints.length - 1].isSegmentEnd = true;
            
            waypoints = newWaypoints;
            selectedWpIndex = null;
            updateTrajectory();
            rebuildWaypointsUI();
        }

    } catch (e) {
        alert("Failed to parse Java code. Verify format matches: new SnapPathBuilder(...).lineTo(...).build()");
        console.error(e);
    }
}

// Clean syntax-agnostic parser that matches matching parenthesis strings
function parseFluentChains(code) {
    const cleanCode = code
        .replace(/\/\*[\s\S]*?\*\//g, '')  
        .replace(/\/\/.*/g, '');           

    let methods = [];
    let i = 0;
    while (i < cleanCode.length) {
        if (cleanCode[i] === '.') {
            i++;
            let name = "";
            while (i < cleanCode.length && /[a-zA-Z0-9_]/.test(cleanCode[i])) {
                name += cleanCode[i];
                i++;
            }
            while (i < cleanCode.length && /\s/.test(cleanCode[i])) {
                i++;
            }
            if (cleanCode[i] === '(') {
                i++;
                let startPos = i;
                let level = 1;
                while (i < cleanCode.length && level > 0) {
                    if (cleanCode[i] === '(') level++;
                    else if (cleanCode[i] === ')') level--;
                    i++;
                }
                if (level === 0) {
                    let args = cleanCode.substring(startPos, i - 1);
                    methods.push({ name, args });
                }
            }
        } else {
            i++;
        }
    }
    return { cleanCode, methods };
}

function parseHeadingToken(token) {
    token = token.trim();
    const radRegex = /Math\.toRadians\(\s*(-?\d+\.?\d*)\s*\)/i;
    const radMatch = token.match(radRegex);
    if (radMatch) {
        return parseFloat(radMatch[1]);
    }
    const num = parseFloat(token);
    if (!isNaN(num)) {
        if (Math.abs(num) <= 2 * Math.PI && num !== 0) {
            return num * 180 / Math.PI;
        }
        return num;
    }
    return 0;
}

// Playback Simulation Control Loop
function toggleSimulation() {
    if (simulation.isPlaying) {
        pauseSimulation();
    } else {
        startSimulation();
    }
}

function startSimulation() {
    if (smoothPathData.length < 2) return;

    // Auto-restart from the beginning if the simulation is at or near the end
    if (simulation.progress >= 99.9 || simulation.currentTime >= estimatedTotalTime - 0.01) {
        simulation.currentTime = 0;
        simulation.progress = 0;
        timelineSlider.value = 0;
    }

    simulation.isPlaying = true;
    simStateVal.textContent = "RUNNING";
    playIcon.style.display = 'none';
    pauseIcon.style.display = 'block';

    robotShadow.style.display = 'block';
    robotLive.style.display = 'block';

    simulation.lastTime = performance.now();
    simulation.animationFrameId = requestAnimationFrame(simulationStep);
}

function pauseSimulation() {
    simulation.isPlaying = false;
    simStateVal.textContent = "PAUSED";
    playIcon.style.display = 'block';
    pauseIcon.style.display = 'none';
    if (simulation.animationFrameId) {
        cancelAnimationFrame(simulation.animationFrameId);
    }
}

function simulationStep(timestamp) {
    if (!simulation.isPlaying) return;

    const dt = (timestamp - simulation.lastTime) / 1000.0 * simulation.speed;
    simulation.lastTime = timestamp;

    simulation.currentTime += dt;

    if (simulation.currentTime >= estimatedTotalTime) {
        simulation.currentTime = estimatedTotalTime;
        simulation.progress = 100;
        timelineSlider.value = 100;
        pauseSimulation();
        simStateVal.textContent = "FINISHED";
        renderRobotSimulationFrame();
        return;
    }

    const progressPercent = (simulation.currentTime / estimatedTotalTime) * 100;
    simulation.progress = progressPercent;
    timelineSlider.value = progressPercent;

    renderRobotSimulationFrame();

    if (simulation.isPlaying) {
        simulation.animationFrameId = requestAnimationFrame(simulationStep);
    }
}

function renderRobotSimulationFrame() {
    if (smoothPathData.length === 0) return;

    const targetTime = (simulation.progress / 100) * estimatedTotalTime;
    simulation.currentTime = targetTime;

    let index = 0;
    while (index < smoothPathData.length - 1) {
        const pt = smoothPathData[index];
        // If the target time is within this point's pause duration, wait here!
        if (targetTime >= pt.time && targetTime <= pt.time + pt.pauseDuration) {
            break;
        }
        // If target time is after this point's pause but before the next point
        if (targetTime > pt.time + pt.pauseDuration && targetTime < smoothPathData[index + 1].time) {
            break;
        }
        index++;
    }

    const pt1 = smoothPathData[index];
    let x, y, heading;

    if (index >= smoothPathData.length - 1) {
        x = pt1.x;
        y = pt1.y;
        heading = pt1.heading;
    } else {
        const pt2 = smoothPathData[index + 1];
        if (targetTime <= pt1.time + pt1.pauseDuration) {
            // stationary during pause
            x = pt1.x;
            y = pt1.y;
            heading = pt1.heading;
        } else {
            // interpolate between end of pause and start of next point
            const t = (targetTime - (pt1.time + pt1.pauseDuration)) / (pt2.time - (pt1.time + pt1.pauseDuration));
            x = pt1.x + t * (pt2.x - pt1.x);
            y = pt1.y + t * (pt2.y - pt1.y);
            heading = pt1.heading + t * normalizeAngleDeg(pt2.heading - pt1.heading);
        }
    }

    const svgCoords = userToSvg(x, y);
    robotLive.setAttribute('transform', `translate(${svgCoords.x}, ${svgCoords.y}) rotate(${heading - 90})`);
    robotLive.style.display = 'block';

    if (selectedWpIndex !== null && selectedWpIndex < waypoints.length) {
        const wp = waypoints[selectedWpIndex];
        const shadowCoords = userToSvg(wp.x, wp.y);
        robotShadow.setAttribute('transform', `translate(${shadowCoords.x}, ${shadowCoords.y}) rotate(${wp.heading - 90})`);
        robotShadow.style.display = 'block';
    } else {
        robotShadow.style.display = 'none';
    }
}
