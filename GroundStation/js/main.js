const socket = io();

// VARIALBES FROM BOAT TELEMETRY TO BE UPDATED VIA WEBSOCKET
let cmdLock = false;
let lastHeading = 0;
let prevWinchDir = 'stop';
let heading = 0; // degrees from North (eg 180 = South)
let l_thrust = 0; // % of max thrust
let r_thrust = 0; // % of max thrust
let boat_lat = 0; // Latitude of boat from onboard GPS
let boat_lng = 0; // Longitude * *
let base_lat = 0;
let base_lng = 0;
let markerIdx = 0;

let battery = 0; // Battery charge level in %
let depth = 0; // function of winch rotation angle
let speed = 0; // m/s from GPS
let acceleration = 0; // m/s^2 from IMU
let roll = 0; // degrees from IMU
let pitch = 0; // degrees from IMU
let omega = 0; // angular velocity from IMU
let pwrLim = 50;
let status = false;
let headGains = {'Kp': 0.0, 'Ki': 0.0, 'Kd': 0.0, 'N': 0.0};
let prev_headGains = {'Kp': 0.0, 'Ki': 0.0, 'Kd': 0.0, 'N': 0.0};
let posGains = {'Kp': 0.0, 'Ki': 0.0, 'Kd': 0.0, 'N': 0.0};
let prev_posGains = {'Kp': 0.0, 'Ki': 0.0, 'Kd': 0.0, 'N': 0.0};

let manNavMode = true; // settings toggle for manual or auto navigation
let moveMarker = false; // flag for if the Move marker menu option has been selected

// State/Mode Change Flags \\
let autoFlag = false;
let holdFlag = false;
let pwrFlag = false;
let manualFlag = false;
// ======================== \\


// Body Elements
const totalOverlay = document.getElementById('body-overlay');

// Header Elements
const connectionStatus = document.getElementById('connection-status');
const rssi = document.getElementById('rssi');
const settingsIcon = document.getElementById("settings-icon");
const clockDiv = document.getElementById("clock");

// Map & Switch Elements
const switch1 = document.getElementById("switch1");
const switch2 = document.getElementById("switch2");
const switch3 = document.getElementById("switch3");
const modeTxt = document.getElementById("mode-txt");
const distTxt = document.getElementById("dist-txt");
const posTxt = document.getElementById("pos-txt");
const coordBox = document.getElementById('curr-coords-box');
const clearBtn = document.getElementById('clear-map');
const sendBtn = document.getElementById('send-btn');
const computeBtn = document.getElementById('compute-path');
const prg1 = document.getElementById('prg1');
const prg2 = document.getElementById('prg2');
const prg3 = document.getElementById('prg3');
const mapHelp = document.getElementById("map-help-txt");
const mapOverlay = document.getElementById("map-overlay-inner-div");


// Marker Menu Elements
const menu = document.getElementById('context-menu');
const menuLoop = document.getElementById('menu-loop');
const menuEdit = document.getElementById('menu-edit');
const latInput = document.getElementById('edit-lat');
const lonInput = document.getElementById('edit-lon');
const menuMove = document.getElementById('menu-move');
const saveCoordsBtn = document.getElementById('save-coords-btn');


// Info Box Elements
const downloadBtn = document.getElementById('download-csv-btn');
const info_boatGraphic = document.getElementById('inner-boat-graphic');
const tarHeadingTxt = document.getElementById('tar-heading-txt');
const info_heading = document.getElementById('heading-txt');
const tarDistTxt = document.getElementById('tar-dist-txt');
const lThrustText = document.getElementById('l-thrust-per');
const rThrustText = document.getElementById('r-thrust-per');
const chargePer = document.getElementById('chrg-lvl');
const chargeBar = document.getElementById('charge-bar')
const speedText = document.getElementById('speed-txt');
const accelText = document.getElementById('accel-txt');
const rollText = document.getElementById('roll-txt');
const pitchText = document.getElementById('pitch-txt');
const omegaText = document.getElementById('omega-txt');

const speed_elm = document.getElementById('speed');
const accel_elm = document.getElementById('accel');
const roll_elm = document.getElementById('roll');
const pitch_elm = document.getElementById('pitch');
const omega_elm = document.getElementById('omega');



// Control Box Elements
const speedLim = document.getElementById("speed-limiter");
const speedLimBox = document.getElementById("speed-box");
const speedLimHandle = document.getElementById("speed-box-handle");
const speedLimTxt = document.getElementById("speed-lim-text");
const fwdBtn = document.getElementById("forward"); 
const backBtn = document.getElementById("backward");
const leftBtn = document.getElementById("left");
const rightBtn = document.getElementById("right");
const stopBtn = document.getElementById("stop");



function setPwrLim(lim) {
    speedLimBox.style.height = lim + "%";
    speedLimTxt.textContent = Math.round(lim) + "%";
    pwrLim = lim;
}

            
socket.on('load_config', function(data) {
    pwrFlag = toggleSwitch(switch3, data.pwr_mode);
    holdFlag = toggleSwitch(switch2, data.hold_mode);
    autoFlag = toggleSwitch(switch1, data.auto_mode);
    updateModeText();
    toggleDarkMode(data.dark_mode);
    updateGains(data.head_gains, data.pos_gains);
    setPwrLim(data.pwrLim);
    base_lat = data.base_lat;
    base_lng = data.base_lon;
    boat_lat = data.boat_lat;
    boat_lng = data.boat_lon;
    radius = data.radius;
    baseLatInput.placeholder = base_lat + "° (Lat)"
    baseLonInput.placeholder = base_lng + "° (Lon)"
    maxWinchDepthInput.placeholder = data.max_winch_depth + "ft"
    setupMap(base_lat, base_lng, boat_lat, boat_lng, radius);
    updateWaypoints(data.waypoints);
    navModeToggle();
    toggleMapOverlay();
});

/* ========================================================================================================
===========================================================================================================
======================================================================================================== */

let map = L.map('map').setView([boat_lat, boat_lng], 16);
// Street Map URL (online only): https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png
// Local Map URL: /map-tiles/{z}/{x}/{y}

const transparentPixel = 'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNkYAAAAAYAAjCB0C8AAAAASUVORK5CYII=';

const regionalLayer = L.tileLayer('/map-tiles/{z}/{x}/{y}?v=2', {
    minZoom: 10,
    maxZoom: 19,
    maxNativeZoom: 16 // Automatically stretches
}).addTo(map);

const highResLayer = L.tileLayer('/map-tiles/{z}/{x}/{y}?v=2', {
    minZoom: 17,
    maxZoom: 19,
    errorTileUrl: transparentPixel // If Python returns 404, become invisible
}).addTo(map);

let layerControl = L.control.layers().addTo(map);
const baseIcon = L.icon({
    iconUrl: '/static/images/GroundStation.png',
    iconSize: [26, 50],
});
const base_marker = L.marker([base_lat, base_lng], { 
    icon: baseIcon 
}).addTo(map);
base_marker.bindPopup("Base Station").openPopup();


const circle = L.circle([base_lat, base_lng], {
    color: 'red',
    fillColor: '#f03',
    fillOpacity: 0.05,
    radius: 0
    }).addTo(map);
layerControl.addOverlay(circle, "Max Range");

const boatIcon = L.icon({
    iconUrl: '/static/images/BoatIcon.png',
    iconSize: [40, 50],
    iconAnchor: [20, 25],
    popupAnchor: [0, 0],
});
let boatMarker = L.marker([boat_lat, boat_lng], {
    icon: boatIcon,
    rotationAngle: heading,
    rotationOrigin: 'center center'
}).addTo(map);
boatMarker.bindPopup("Boat Position")


const SnapControl = L.Control.extend({
    options: { position: 'topright' }, 
    onAdd: function (map) {
        const container = L.DomUtil.create('div', 'leaflet-bar leaflet-control');
        container.style.backgroundColor = 'white';
        container.style.display = 'flex';
        container.style.flexDirection = 'column';
        L.DomEvent.disableClickPropagation(container);
        const btnBoat = L.DomUtil.create('a', '', container);
        btnBoat.innerHTML = '🚤'; // You can use text or FontAwesome icons here
        btnBoat.href = '#';
        btnBoat.title = 'Snap to Boat';
        btnBoat.style.fontSize = '20px';
        btnBoat.style.lineHeight = '30px';
        btnBoat.style.textAlign = 'center';
        btnBoat.style.cursor = 'pointer';
        const btnBase = L.DomUtil.create('a', '', container);
        btnBase.innerHTML = '🏠';
        btnBase.href = '#';
        btnBase.title = 'Snap to Base';
        btnBase.style.fontSize = '20px';
        btnBase.style.lineHeight = '30px';
        btnBase.style.textAlign = 'center';
        btnBase.style.cursor = 'pointer';

        L.DomEvent.on(btnBoat, 'click', function(e) {
            L.DomEvent.preventDefault(e); // Stops the browser from scrolling up
            map.flyTo([boat_lat, boat_lng], 16); 
        });

        L.DomEvent.on(btnBase, 'click', function(e) {
            L.DomEvent.preventDefault(e);
            map.flyTo([base_lat, base_lng], 16);
        });
        return container;
    }
});

// 4. Add the control to your map
map.addControl(new SnapControl());

function setupMap(base_lat, base_lon, boat_lat, boat_lon, radius) {
    map.setView([base_lat, base_lon], 16);
    boatMarker.setLatLng([boat_lat, boat_lon]);
    base_marker.setLatLng([base_lat, base_lon]);
    circle.setLatLng([base_lat, base_lon]);
    circle.setRadius(radius);
}

let markerArray = [];
let startMarker = null;
let endMarker = null;
let markerIndex = 0;
let markerLines = [];
let waypoints_layer = L.layerGroup().addTo(map)
let activeMarker = null;

function updateWaypoints(waypoints) {
    clearWaypoints(false);
    markerArray = waypoints.map(point => {
        let marker = L.marker([point.lat, point.lon]);
        marker.addTo(waypoints_layer);
        addMarkerContextMenu(marker);
        return marker; 
    });
    markerIndex = markerArray.length;
}   


function addMarkerContextMenu(marker) {
    marker.on('contextmenu', function (e2) {
            if (marker == markerArray[0]) {
                menuLoop.style.color = 'black';
            } else {
                menuLoop.style.color = 'grey';
            }
            L.DomEvent.stopPropagation(e2);
            activeMarker = marker;
            menu.classList.remove('hidden');
            menu.style.left = e2.originalEvent.pageX + 'px';
            menu.style.top = e2.originalEvent.pageY + 'px';
        });     
}

function onMapClick(e) {
    if (!moveMarker) {    
        let marker = L.marker([e.latlng.lat, e.latlng.lng])
        closeMarkerMenu();
        let canProceed = pathPlanner();
        if (canProceed) {
            markerArray.push(marker);
            marker.addTo(waypoints_layer);
            addMarkerContextMenu(marker);
            markerIndex++;
            redrawLines();
            pathPlanner();
        }  
    }
}
map.on("click", onMapClick);


function pathPlanner() {
    if (manNavMode) {
        mapHelp.textContent = "";
        return true;
    }

    if (markerIndex == 0) {
        mapHelp.textContent = "Select Start Point";
        return true;
    } else if (markerIndex == 1) {
        mapHelp.textContent = "Select End Point";
        return true;
    }
    mapHelp.textContent = "";
    return false;
}



/* For Coordinate readout in lwr right corner */
map.on('mousemove', (e) => {
    let lat = e.latlng.lat.toFixed(6);
    let lon = e.latlng.lng.toFixed(6);
    coordBox.querySelector('span:nth-child(1)').textContent = lat;
    coordBox.querySelector('span:nth-child(2)').textContent = lon;
})

const editCoords = document.getElementById('edit-coords');
// Remove Marker
document.getElementById('menu-remove').addEventListener('click', () => {
    if (activeMarker) {
        if (activeMarker == markerArray[0] && markerArray[0] == markerArray[markerArray.length - 1]) {
            markerArray.pop();
        }
        waypoints_layer.removeLayer(activeMarker);
        markerArray = markerArray.filter(m => m !== activeMarker);
        markerIndex--;
        menu.classList.add('hidden');
        pathPlanner();
        redrawLines();
    }
})

// Close Menu
document.getElementById('menu-close').addEventListener('click', () => {
    closeMarkerMenu();
})

function closeMarkerMenu() {
    menu.classList.add('hidden');
    editCoords.classList.add('hidden');
}

// Edit Marker Coordinates
menuEdit.addEventListener('click', () => {
    let rect = menuEdit.getBoundingClientRect();
    editCoords.style.left = (rect.right + window.scrollX) + 'px';
    editCoords.style.top = (rect.top + window.scrollY) + 'px';
    editCoords.classList.remove('hidden');
    latInput.placeholder = activeMarker.getLatLng().lat.toFixed(6);
    lonInput.placeholder = activeMarker.getLatLng().lng.toFixed(6);
})

saveCoordsBtn.addEventListener('click', () => {
    let newLat = latInput.value;
    let newLng = lonInput.value;
    if (newLat && newLng) {
        activeMarker.setLatLng([newLat, newLng]);
    }
    closeMarkerMenu();
    redrawLines();
})

// Move (Replace) Marker
menuMove.addEventListener('click', () => {
    if (activeMarker) {
        closeMarkerMenu();
        moveMarker = true;
        map.on('mousemove', markerFollowMouse);
        map.once('click', function(e) {
            activeMarker.setLatLng(e.latlng);
            moveMarker = false;
            map.off('mousemove', markerFollowMouse);
        })
    }
    redrawLines();
});

function markerFollowMouse(e) {
    activeMarker.setLatLng(e.latlng);
    redrawLines();
}

// Create Loop
menuLoop.addEventListener('click', () => {
    if (activeMarker && activeMarker == markerArray[0] && markerArray.length > 1) {
        let lastMarker = markerArray[markerArray.length - 1];
        let line = L.polyline([lastMarker.getLatLng(), activeMarker.getLatLng()], { color: 'blue' });
        line.addTo(waypoints_layer);
        markerArray.push(activeMarker);
        markerLines.push(line);
        menu.classList.add('hidden');
        redrawLines();
    }
})

let wayIdx = 0;
function redrawLines() {
    waypoints_layer.clearLayers();
    markerLines = [];
    lineColor = 'blue';

    if (markerArray.length > 1) {
        let pt1 = boatMarker.getLatLng();
        let pt2 = markerArray[wayIdx].getLatLng();
        let dist = pt1.distanceTo(pt2);
        if (autoFlag == true && dist < 3) {
            wayIdx++;
        }
    }
    if (markerArray[0] == markerArray[markerArray.length - 1] && markerArray.length > 1) {
        lineColor = 'red';
    }
    for (let i = 0; i < markerArray.length; i++) {
        let marker = markerArray[i];
        marker.addTo(waypoints_layer);
        if (i == wayIdx) {
            let line = L.polyline([boatMarker.getLatLng(), markerArray[i].getLatLng()], { color: 'green' });
            line.addTo(waypoints_layer);
            markerLines.push(line);
            line.addEventListener("contextmenu", lineClick)
        }

        if (i > wayIdx) {
            let line = L.polyline([markerArray[i - 1].getLatLng(), marker.getLatLng()], { color: lineColor });
            line.addTo(waypoints_layer);
            markerLines.push(line);
            line.addEventListener("contextmenu", lineClick)
        }
    }
}


function lineClick(e) {
    let marker = L.marker([e.latlng.lat, e.latlng.lng])
    marker.addTo(waypoints_layer);
}
layerControl.addOverlay(waypoints_layer, "Waypoints");


/* =================CLEAR MAP & SEND WAYPOINTS BEHAVIOR======================== */
let holdTimer;
const holdDuration = 500; // ms, = 0.5 seconds

function startHold(progress, button, setter) {
    progress.style.transition = `width ${holdDuration}ms linear`;
    button.style.scale = '0.99';
    progress.style.width = '100%';

    holdTimer = setTimeout(() => {
        executeHold(setter)
    }, holdDuration);
}

function cancelHold(progress, button) {
    progress.style.transition = 'width 0.2s ease-out';
    button.style.scale = '1';
    progress.style.width = '0%';
    clearTimeout(holdTimer);
}

function executeHold(type) {
    if (type == 'clear') {
        clearWaypoints(true);
    } else if (type == 'send') {
        sendGPS_Command(markerArray);
    } else {
        if (markerArray.length == 2) {
            let startLatLng = markerArray[0].getLatLng();
            let endLatLng = markerArray[1].getLatLng();
            let pathPoints = {
                start_lat: startLatLng.lat,
                start_lng: startLatLng.lng,
                end_lat: startLatLng.lat,
                end_lng: endLatLng.lng
            }
            socket.emit('compute-path', pathPoints);
        }
    }
}



function clearWaypoints(sendToServer) {
    wayIdx = 0; // index for boat 'popping' waypoints off the stack
    markerIndex = 0; // to figure out where to place the marker in the array
    waypoints_layer.clearLayers()
    markerArray = [];
    markerLines = [];
    if (sendToServer) { clearGPS_Command(); }
}

clearBtn.addEventListener('mousedown', () => startHold(prg1, clearBtn, 'clear'));
clearBtn.addEventListener('mouseup', () => cancelHold(prg1, clearBtn));
clearBtn.addEventListener('mouseleave', () => cancelHold(prg1, clearBtn));

sendBtn.addEventListener('mousedown', () => startHold(prg2, sendBtn, 'send'));
sendBtn.addEventListener('mouseup', () => cancelHold(prg2, sendBtn));
sendBtn.addEventListener('mouseleave', () => cancelHold(prg2, sendBtn));

computeBtn.addEventListener('mousedown', () => {
    if (markerArray.length > 1) {
        startHold(prg3, computeBtn, 'path');
    }
});
computeBtn.addEventListener('mouseup', () => cancelHold(prg3, computeBtn));
computeBtn.addEventListener('mouseleave', () => cancelHold(prg3, computeBtn));
/* ===========================================================================*/

/* =================POWER LIMITER BEHAVIOR========================*/
speedLimHandle.addEventListener('mousedown', function(e) {
    e.preventDefault();
    window.addEventListener('mousemove', dragInternalBox);
    window.addEventListener("mouseup", stopInternalDrag);
});
function dragInternalBox(e) {
    // 1. Get the bounding box of the container
    const rect = speedLim.getBoundingClientRect();
    // 2. Calculate the distance from the bottom of the container to the mouse
    let newHeight = rect.bottom - e.clientY;
    // 3. Constrain the height so it doesn't go outside the frame
    if (newHeight < rect.height*0.1) {
        newHeight = 0.1*rect.height;
    } else if  (newHeight > rect.height) {
        newHeight = rect.height;
    }
    // 4. Apply the height to the inner box
    speedLimBox.style.height = newHeight + 'px';

    // 5. Update the text percentage
    let percent = Math.round((newHeight / rect.height) * 100);
    speedLimTxt.textContent = percent + "%";
    pwrLim = percent;
}
function stopInternalDrag() {
    window.removeEventListener('mousemove', dragInternalBox);
}
/* ===============================================================*/

/*=================Switch Behavior========================*/

// set flag = true to turn on the switch, flag = false to turn off the switch
// turning off switch3 (pwr switch) will cause all other switches to turn off
function toggleSwitch(element, flag) {
    let id = element.id;
    let p = document.querySelector(`#${id} p`);
    if (pwrFlag == false && id != "switch3") {
        flag = turnOff(element, p)
    } else if (id == "switch3") {
        if (flag) { flag = turnOn(element, p); }            
        else { flag = turnOffAll(); }
    } else {
        if(flag) { flag = turnOn(element, p); }
        else { flag = turnOff(element, p); }
    }
    return flag;
}

function turnOff(element, p) {
    element.style.transform = 'translateY(50px)';
    element.style.backgroundColor = 'red';
    p.textContent = "O";
    updateModeText();
    return false;
}

function turnOffAll() {
    autoFlag = turnOff(switch1, document.querySelector(`#${switch1.id} p`));
    holdFlag = turnOff(switch2, document.querySelector(`#${switch2.id} p`));
    pwrFlag =  turnOff(switch3, document.querySelector(`#${switch3.id} p`));
    return false;
}

function turnOn(element, p) {
    element.style.transform = 'translateY(0px)';
    element.style.backgroundColor = 'blue';
    p.textContent = "I";
    return true;
}

function updateModeText() {
    if (manualFlag == true) {
        modeTxt.textContent = "MANUAL";
        modeTxt.style.color = 'green';
    } else if (holdFlag == true) {
        modeTxt.textContent = "HOLD";
        modeTxt.style.color = 'red';
    } else if (autoFlag == true) {
        modeTxt.textContent = "AUTO";
        modeTxt.style.color = 'blue';
    } else if (pwrFlag == false) {
        modeTxt.textContent = "OFF";
        modeTxt.style.color = 'black';  
    } else {
        modeTxt.textContent = "IDLE";
        modeTxt.style.color = 'black';
    }
}

switch1.addEventListener('click', () => {
   autoFlag = toggleSwitch(switch1, !autoFlag);
   updateModeText();
   sendControl_Mode();
});


switch2.addEventListener('click', () => {
    holdFlag = toggleSwitch(switch2, !holdFlag);
    updateModeText();
    sendControl_Mode();
});


switch3.addEventListener('click', () => {
    pwrFlag = toggleSwitch(switch3, !pwrFlag);   
    updateModeText();
    sendControl_Mode();
    
});
/*====================================================================*/

function updateClock() {
    const now = new Date();
    const hours = String(now.getHours()).padStart(2, '0');
    const minutes = String(now.getMinutes()).padStart(2, '0');
    const seconds = String(now.getSeconds()).padStart(2, '0');
    const timeString = `${hours}:${minutes}:${seconds}`;
    clockDiv.textContent = timeString;
    return timeString;
}
updateClock();
setInterval(updateClock, 1000);


const controlsCont = document.getElementById('inner-controls-cont');
const keyboardIcon = document.getElementById('keyboard-icon');
let controlArmed = false;
let keyLocks = {};

keyboardIcon.addEventListener('click', (e)=>  {
    e.stopPropagation();
    if (controlArmed) {
        controlsCont.blur();
        document.getElementById('key-warn-txt').classList.add("hidden");
    } else {
        controlsCont.focus();
        document.getElementById('key-warn-txt').classList.remove("hidden");
    }
    controlArmed = !controlArmed    
})

document.addEventListener('mousedown', (e) => {
    if (keyboardIcon.contains(e.target)) return;
    if (document.activeElement === controlsCont && !keyboardIcon.contains(e.target)) {
        if (!controlsCont.contains(e.target)) {
            e.preventDefault(); 
        }
    }
});

controlsCont.addEventListener('keydown', keyPressed);
let activeKey = null;

function keyPressed(e) {
    if (!controlArmed) { return }
    const key = e.key.toLowerCase();
    if (['w', 'a', 's', 'd', 'r', 'f', ' '].includes(key)) {
        if (activeKey != null) return;
        activeKey = key;
        e.preventDefault();
        switch(key) {
            case 'w': move('forward'); break;
            case 's': move('backward'); break;
            case 'a': move('left'); break;
            case 'd': move('right'); break;
            case 'r': sendWinchCommand('up'); break;
            case 'f': sendWinchCommand('down'); break;
            case ' ': move('stop'); break;
        }
    } else if (['shift', 'control'].includes(key)) {
        e.preventDefault();
        if ((pwrLim <= 99.5 && key == 'shift') || (pwrLim >= 10.5 && key == 'control')) {
            switch(key) {
                case 'shift': setPwrLim(pwrLim + 0.5); break;
                case 'control': setPwrLim(pwrLim - 0.5); break;
            }
        }
    }
}

controlsCont.addEventListener('keyup', (e) => {
    const key = e.key.toLowerCase();
    if (activeKey == key) {
        activeKey = null;
        if (['w', 'a', 's', 'd', 'r', 'f', ' '].includes(key)) {
            e.preventDefault();   
            if(['w', 'a', 's', 'd', ' '].includes(key)) {
                coast();
                switch(key) {
                    case 'w': fwdBtn.classList.remove("active"); break;
                    case 's': backBtn.classList.remove("active"); break;
                    case 'a': leftBtn.classList.remove("active"); break;
                    case 'd': rightBtn.classList.remove("active"); break;
                    case ' ': stopBtn.classList.remove('active'); break;
                    case 'r': sendWinchCommand('stop'); break;
                    case 'f': sendWinchCommand('stop'); break;
                }
            } else {
                sendWinchCommand('stop');
            }
        }
    }
});


function move(direction) {
    if (!cmdLock) {
        sendThrustCommand(direction);
        switch(direction) {
            case 'forward': fwdBtn.classList.add("active"); break;
            case 'backward': backBtn.classList.add("active"); break; 
            case 'left': leftBtn.classList.add("active"); break; 
            case 'right': rightBtn.classList.add("active"); break;
            case 'stop': stopBtn.classList.add("active"); break;
        }
        manualFlag = true;
        updateModeText();
    }
    
}

function coast() {
    if (!cmdLock) {
        sendThrustCommand('coast');
        manualFlag = false;
        updateModeText();    
    }
    
}

fwdBtn.addEventListener('mousedown', function(e) {
    move('forward');
});
fwdBtn.addEventListener('mouseup', function(e) {
    coast();
    fwdBtn.classList.remove("active");
})
backBtn.addEventListener('mousedown', function(e) {
    move('backward');
});
backBtn.addEventListener('mouseup', function(e) {
   coast();
   backBtn.classList.remove("active");
});
leftBtn.addEventListener('mousedown', function(e) {
    move('left');
});
leftBtn.addEventListener('mouseup', function(e) {
    coast();
    leftBtn.classList.remove("active");
})
rightBtn.addEventListener('mousedown', function(e) {
    move('right');
});
rightBtn.addEventListener('mouseup', function(e) {
   coast();
   rightBtn.classList.remove("active");

})
stopBtn.addEventListener('mousedown', function(e) {
    coast();
});
stopBtn.addEventListener('mouseup', function(e) {
    coast();
    stopBtn.classList.remove("active");
})



function updateConnectionStatus(element, status) {
    if (status == true) {
        element.textContent = "Connected";
        element.style.color = "lightgreen";
        if (!darkModeToggle.checked) {
            element.style.color = 'green';
        }
    } else {
        element.textContent = "Disconnected";
        element.style.color = "red";
    }
}

/* ====================COMMAND SENDING BEHAVIOR========================*/
function sendThrustCommand(direction) {
    socket.emit('drive_command', {
        dir: direction,
        pwr: pwrLim
    });
}

function setWinchSpeed(dir) {
    if (prevWinchDir == dir) {
        return 1;
    } else {
        prevWinchDir = dir;
        return 0;
    }
}

function sendWinchCommand(dir) {
    if (!cmdLock) {
        if (dir == 'up' || dir == 'down') {
            if (dir == 'up') { 
                upBtn.classList.add("active"); 
                upBtn.style.color = 'red';
                dwnBtn.style.color = 'black';
            } 
            else if (dir == 'down') { 
                dwnBtn.classList.add("active"); 
                dwnBtn.style.color = 'red';
                upBtn.style.color = 'black';
            }
            document.getElementById("winch-controls").classList.add("is-pulsating");
        } else {
            winchStopBtn.classList.add("active");
            dwnBtn.style.color = 'black';
            upBtn.style.color = 'black';
            document.getElementById("winch-controls").classList.remove("is-pulsating");
        }
        speed = setWinchSpeed(dir);
        socket.emit('winch_command', {
            direction: dir,
            speed: speed
        });
    }
}

function sendGPS_Command(waypoints_arr) {
    let waypointsDict = [];
    for (let i = 0; i < waypoints_arr.length; i++) {
        let latLng = waypoints_arr[i].getLatLng();
        waypointsDict.push({
            lat: latLng.lat,
            lon: latLng.lng
        });
    }
    cmdLock = true;
    totalOverlay.classList.remove('hidden');
    socket.emit('receive_GPS_coords', { waypoints: waypointsDict });
}

function clearGPS_Command() {
    socket.emit('clear_waypoints')
}

function sendControl_Mode() {
    totalOverlay.classList.remove('hidden');
    socket.emit('control_mode', {
        pwr: pwrFlag,
        auto: autoFlag,
        hold: holdFlag
    });

}
/* =================================================================== */
downloadBtn.addEventListener('click', ()=> {
    window.location.href = '/download-csv';
});


let currentRotation = 0; // Keep track of the total absolute rotation
function updateHeadingInfo(heading, tarHeading, tarDist) {
    let delta = heading - (currentRotation % 360);
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;
    currentRotation += delta;
    let headingTxt = heading.toString().padStart(3, '0');
    let tarHeading_Txt = tarHeading.toString().padStart(3, '0');
    info_heading.textContent = headingTxt + "°";
    if (autoFlag || holdFlag) { 
        tarHeadingTxt.textContent = tarHeading_Txt + "°"; 
        tarDistTxt.textContent = `DIST: ${tarDist}m`;
    } else {
        tarHeadingTxt.textContent = "";
        tarDistTxt.textContent = "";
    }
    boatMarker.setRotationAngle(heading);
    info_boatGraphic.style.transform = `rotate(${currentRotation}deg)`;    
}

function updateBatteryInfo(soc) {
    chargeBar.style.width = `${soc}%`;
    chargePer.textContent = `${soc}%`;
    if (soc > 50) {
        chargeBar.style.backgroundColor = 'greenyellow';
    } else if (soc > 20 && soc <= 50) {
        chargeBar.style.backgroundColor = 'yellow';
    } else if (soc <= 20) {
        chargeBar.style.backgroundColor = 'red';
    }
}


function updatePositionInfo(lat, lon, dist) {
    boat_lat = lat;
    boat_lng = lon;
    boatMarker.setLatLng([lat, lon]);
    distTxt.textContent = `${dist.toFixed(0)}m`;
    posTxt.textContent = `(${lat.toFixed(4)}, ${lon.toFixed(4)})`;
    window.lat = lat;
    window.lon = lon;
    redrawLines();
}


function updateThrustInfo(l_thrust, r_thrust) {
    if (l_thrust < 0) {
        document.getElementById('lwr-l-thrust-bar').style.height = `${-l_thrust}%`;
        document.getElementById('upr-l-thrust-bar').style.height = `0%`;
    } else {
        document.getElementById('upr-l-thrust-bar').style.height = `${l_thrust}%`;
        document.getElementById('lwr-l-thrust-bar').style.height = `0%`;
    }

    if (r_thrust < 0) {
        document.getElementById('lwr-r-thrust-bar').style.height = `${-r_thrust}%`;
        document.getElementById('upr-r-thrust-bar').style.height = `0%`;
    } else {
        document.getElementById('upr-r-thrust-bar').style.height = `${r_thrust}%`;
        document.getElementById('lwr-r-thrust-bar').style.height = `0%`;
    }
    lThrustText.textContent = `${l_thrust}`;
    rThrustText.textContent = `${r_thrust}`;

}

function updateDepthInfo(depth_val) {
    let prefix = "";
    if (depth_val < 9) {
        prefix = "0";
    }
    depthText.textContent = prefix + depth_val.toFixed(1);
}

function updateRSSI(rssi_val) {
    if (connectionStatus.textContent == 'Connected') {
            rssi.textContent = rssi_val;
            document.getElementById('status-p2').classList.remove('is-collapsed');
        if (darkModeToggle.checked) {
            if (rssi_val < -95) {
                rssi.style.color = 'red';
            } else if (rssi_val < -50) {
                rssi.style.color = 'yellow';
            } else {
                rssi.style.color = 'lightgreen';
            }
        } else {
            if (rssi_val < -95) {
                rssi.style.color = 'red';
            } else if (rssi_val < -50) {
                rssi.style.color = 'darkyellow';
            } else {
                rssi.style.color = 'green';
            }        
        }
    } else {
        document.getElementById('status-p2').classList.add('is-collapsed');
        rssi.textContent = '-';
        rssi.color = 'black';
    }
}


function toggleMapOverlay() {
    if (connectionStatus.textContent == 'Disconnected') {
        mapOverlay.classList.add('is-collapsed');
    } else {
        mapOverlay.classList.remove('is-collapsed');
    }
}

function handleReturn(return_status) {
    // if (return_status) {
    //     document.getElementById('map-overlay-return').style.visibility = 'visible';
    // } else {
    //     document.getElementById('map-overlay-return').style.visibility = 'hidden';
    // }
}

const activeCharts = {};
const activeSeries = {};
/* ====================COMMAND RECEIVING BEHAVIOR========================*/
socket.on('telemetry_update', function(data) {
        updatePositionInfo(data.lat, data.lon, data.dist);
        updateHeadingInfo(data.heading, data.tar_heading, data.tar_dist);
        updateThrustInfo(data.l_thrust, data.r_thrust);
        updateBatteryInfo(data.soc);
        updateConnectionStatus(connectionStatus, data.connection);;
        updateDepthInfo(data.winch_depth);
        speedText.textContent = `${data.speed.toFixed(1)}`;
        accelText.textContent = `${data.acceleration.toFixed(1)}`;
        rollText.textContent = `${data.roll.toFixed(1)}`;
        pitchText.textContent = `${data.pitch.toFixed(1)}`;
        omegaText.textContent = `${data.omega.toFixed(1)}`; 
        updateRSSI(data.rssi);
        toggleMapOverlay();
        handleIncomingErrors(data.error);
        handleReturn(data.return);
        toggleSwitch(switch1, data.auto);
        toggleSwitch(switch2, data.hold);
        toggleSwitch(switch3, data.pwr);
        autoReturnToggle.checked = data.auto_return;
        pwrFlag = data.pwr;
        autoFlag = data.auto;
        holdFlag = data.hold;
        updateModeText();
        const timestamp = new Date().getTime();
        Object.keys(activeCharts).forEach(metric => {
            const newValue = data[metric]
            if (newValue !== undefined) {
                activeSeries[metric].append(timestamp, newValue);
            }
        })
});

socket.on('GPS_waypoint_update', function(data) {
    updateWaypoints(data);
});

socket.on('gain_update', function(data) {
    closeSettings();
    updateGains(data.heading, data.position);
    
});

socket.on('ack-received', () => {
    console.log("ACK RECEIVED");
    totalOverlay.classList.add('hidden');
    cmdLock = false;
});

socket.on('ack-failure', () => {
    console.log("ACK FAILURE");
    totalOverlay.classList.add('hidden');
    const errPopup = document.getElementById('err-txt-map');
    errPopup.classList.add('show');
    errPopup.textContent = "NO ACK. TRY AGAIN."
    setTimeout(() => {
        errPopup.classList.remove('show');
        cmdLock = false;
    }, 5000);
});


/*
======================GRAPHING BEHAVIOR=========================
================================================================
================================================================
*/
speed_elm.addEventListener('click', () => {
    toggleGraph('speed', '#00ff00', "Speed [knots]");
});
accel_elm.addEventListener('click', () => {
    toggleGraph('acceleration', '#00ff00', "Acceleration [m/s^2]");
});
roll_elm.addEventListener('click', () => {
    toggleGraph('roll', '#00ff00', "Roll [°]");
});
pitch_elm.addEventListener('click', () => {
    toggleGraph('pitch', '#00ff00', "Pitch [°]");
});
omega_elm.addEventListener('click', () => {
    toggleGraph('omega', '#00ff00', "Angular Velocity [°/s]");
});
document.getElementById("charge-cont").addEventListener('click', () => {
    toggleGraph('soc', '#00ff00', "State of Charge [%]");
});
document.getElementById('left-thrust-bar-cont').addEventListener('click', () => {
    toggleGraph('l_thrust', '#ff0000', "Left Thrust [%]");

});
document.getElementById('right-thrust-bar-cont').addEventListener('click', () => {
    toggleGraph('r_thrust', '#ff0000', "Right Thrust [%]");
});

document.getElementById('depth-val-cont').addEventListener('click', () => {
    toggleGraph('winch_depth', '#00ffff', "Depth [m]");
})

info_boatGraphic.addEventListener('click', () => {
    toggleGraph('heading',  '#00ff00', 'Heading [°]');
})

function toggleThrustGraph() {
    let chart = toggleGraph('l_thrust', '#ff0000', "Motor Thrust [%]");
}

function toggleGraph(metric, color = '#007bff', title) {
    if (activeCharts[metric]) {
        document.getElementById(`container-${metric}`).remove();
        delete activeCharts[metric];
        delete activeSeries[metric];
        return;
    }
    const grid = document.getElementById('graphs-grid');
    const wrapper = document.createElement('div');
    wrapper.style.height = "333px";
    wrapper.style.width = "100%";
    wrapper.classList.add("glass-card");
    wrapper.id = `container-${metric}`;
    wrapper.innerHTML = `<h4>${title}</h4><canvas id="canvas-${metric}"></canvas>`;
    grid.appendChild(wrapper);

    const chart = new SmoothieChart({
        grid: { strokeStyle: 'rgba(125,125,125,0.2)', verticalSections: 6 },
        labels: { 
            fillStyle: '#ffffff',
            fontSize: 16 
        },
        millisPerPixel: 100, // Controls scroll speed
        responsive: true,
        interpolation: 'bezier'
    });

    const line = new TimeSeries();
    chart.addTimeSeries(line, { strokeStyle: color, fillStyle: color + '33', lineWidth: 2 });
    activeSeries[metric] = line;
    chart.streamTo(document.getElementById(`canvas-${metric}`), 1000); // 1s delay for smoothness
    activeCharts[metric] = chart;
}



/* EXTRA STUFF THAT ISN'T REALLY USED */
fwdBtn.addEventListener('contextmenu', function(e) {
    e.preventDefault();
});

backBtn.addEventListener('contextmenu', function(e) {
    e.preventDefault();
});
leftBtn.addEventListener('contextmenu', function(e) {
    e.preventDefault();
});
rightBtn.addEventListener('contextmenu', function(e) {
    e.preventDefault();
});
stopBtn.addEventListener('contextmenu', function(e) {
    e.preventDefault();
});

const credBtn = document.getElementById("logo");
const credCloseBtn = document.getElementById("credits-close-btn");
const creditCont = document.getElementById("credits-div");

credBtn.addEventListener('click', () => {
    creditCont.classList.toggle("active");
    topCont.classList.toggle("is-active");
    topCont.classList.toggle("disabled-element");
});

credCloseBtn.addEventListener('click', () => {
    creditCont.classList.remove("active");
    topCont.classList.remove("is-active");
    topCont.classList.remove("disabled-element");
})
    