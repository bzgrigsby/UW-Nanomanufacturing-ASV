// Settings Elements
let holdR = 400;

const settingsMenu = document.getElementById("settings-menu");
const topCont = document.getElementById("top-cont");
const settingsCloseBtn = document.getElementById("settings-close-btn");
const darkModeToggle = document.getElementById("dark-mode");
const manNavToggle = document.getElementById("nav-toggle");
const baseLatInput = document.getElementById("base-lat");
const baseLonInput = document.getElementById("base-lon");
const setBaseCoordsBtn = document.getElementById("set-base-coords");
const selectOnMapBtn = document.getElementById("choose-base-coords-map");
const maxWinchDepthInput = document.getElementById("desired-depth");
const setWinchDepthBtn = document.getElementById("set-depth");
const restoreWaypointsBtn = document.getElementById("restore-waypoints");
const headGainsCont = document.getElementById("heading-gains");
const posGainsCont = document.getElementById("position-gains");
const sendGainsBtn = document.getElementById('send-gains');
const restoreGainsBtn = document.getElementById('restore-gains');
const holdRadiusInput = document.getElementById('hold-radius-input');
const holdRadiusBtn = document.getElementById('set-hold-radius');

const autoReturnToggle = document.getElementById('auto-return');
const webserverRestartBtn = document.getElementById('restart-webserver');

const errsDiv = document.getElementById('settings-errors');

holdRadiusBtn.addEventListener('click', () => {
    let input = holdRadiusInput.value.split(' ')[0].trim();
    console.log(input);
    if (input != "") {
        holdR = input;  
        socket.emit('set-hold-radius', holdR/1000.0);
        holdRadiusInput.placeholder = `${holdR} mm`
        holdRadiusInput.value = ""
    } 
});

function createPidSlider(parent, gain, label, min=0, max=100) {
    const id = `${label}-${gain}`;
    let step = (max - min) / 100
    const html = `
        <div class="settings-slide-cont">
            <p>${gain} = <span id="${id}-txt"></span></p>
            <div class="PID-slide-cont">
                <input type="range" 
                    min="${min}" 
                    max="${max}" 
                    step="${step}"
                    class="range-slider" 
                    id="${id}-val">
                <img class="reset-img" id="${id}-reset" src="/static/images/reset-icon.png">
            </div>
        </div>
    `;
    parent.insertAdjacentHTML('beforeend', html);
    const txt = document.getElementById(`${id}-txt`);
    const slider = document.getElementById(`${id}-val`);
    const reset = document.getElementById(`${id}-reset`);
    const getActiveMap = () => (label === 'pos' ? posGains : headGains);
    const getPrevMap = () => (label === 'pos' ? prev_posGains : prev_headGains);

    txt.textContent = getActiveMap()[gain]
    slider.value = getActiveMap()[gain]

    slider.addEventListener('input', () => {
        txt.textContent = slider.value;
        getActiveMap()[gain] = parseFloat(slider.value);
    });
    reset.addEventListener('click', () => {
        slider.value = getPrevMap()[gain]
        txt.textContent = getPrevMap()[gain]
    });
}

const gains = ['Kp', 'Ki', 'Kd', 'N'];
const gainLabels = ['head', 'pos'];
gains.forEach(gain => {
    let min = 1;
    let max = 100;

    if (gain == 'Ki') {
        min = 0.0;
        max = 0.2;
    } else if (gain == 'Kp') {
        min = 0.0;
        max = 20.0;
    } else if (gain == 'Kd') {
        min = 0.0;
        max = 2.0;
    } 
    createPidSlider(posGainsCont, gain, "pos", min, max);
        createPidSlider(headGainsCont, gain, "head", min, max);
    
});

function toggleDarkMode(on) {
    let items = document.getElementsByClassName('glass-card');
    let color = 'black';
    let topCont = document.getElementById('top-cont');
    darkModeToggle.checked = on;
    if (!on) {
        topCont.style.backgroundColor = 'rgb(225, 199, 225)';
        document.querySelector('body').style.backgroundColor = 'white'; 
        topCont.style.boxShadow = '0 4px 32px black';
        color = 'black';
    } else {
        topCont.style.backgroundColor = 'grey';
        document.querySelector('body').style.backgroundColor = 'black';
        topCont.style.boxShadow = '0 4px 32px rgba(255,255,255,0.8)';
        color = 'color: rgb(255, 255, 240)';
        color = 'white';
    }
    for (let i = 0; i < items.length; i++) {
        items[i].style.color = color;
    }
    socket.emit('settings-change', {
        dark_mode: on
    });
}
darkModeToggle.addEventListener('change', () => {
    toggleDarkMode(darkModeToggle.checked)
});


function updateGains(n_headGains, n_posGains) {
    let gainMap = {[gainLabels[0]]: n_headGains, [gainLabels[1]]: n_posGains};
    gains.forEach(gain => {
        id_pos = `pos-${gain}`;
        gainLabels.forEach(label => {
            let id_head = `${label}-${gain}`;
            let slider = document.getElementById(`${id_head}-val`);
            let map1 = gainMap[label];
            slider.value = map1[gain];
            
            let txt = document.getElementById(`${id_head}-txt`);
            txt.textContent = map1[gain];
        });
    });
    prev_headGains = structuredClone(n_headGains);
    prev_posGains = structuredClone(n_posGains);
    headGains = structuredClone(n_headGains);
    posGains = structuredClone(n_posGains);
}

function closeSettings() {
    settingsMenu.classList.remove("active");
    topCont.classList.remove("is-active");
    topCont.classList.remove("disabled-element");
}

settingsCloseBtn.addEventListener('click', () => {
    closeSettings();
})

settingsIcon.addEventListener('click', () => {
    settingsMenu.classList.toggle("active");
    topCont.classList.toggle("is-active");
    topCont.classList.toggle("disabled-element");
});


manNavToggle.addEventListener('change', () => {
    navModeToggle();
});
function navModeToggle() {
    manNavMode = manNavToggle.checked;
    if (markerArray.length > 0 && !manNavToggle.checked) {
        clearWaypoints(true);
    }
    clearWaypoints(false);
    markerIndex = 0;
    if (manNavToggle.checked) {
        computeBtn.classList.add('hidden');
    } else {
        computeBtn.classList.remove('hidden');
        mapHelp.textContent = "Select Start Location";
    }
}


setBaseCoordsBtn.addEventListener('click', () => {
    if (baseLatInput != null && baseLonInput != null) {
        let lat_v = parseFloat(baseLatInput.value.trim());
        let lon_v = parseFloat(baseLonInput.value.trim());
        if (isNaN(lat_v) || isNaN(lon_v)) {
            return;
        }
        socket.emit('set-base-coords', {
            lat: lat_v,
            lon: lon_v
        });
    }
});


selectOnMapBtn.addEventListener('click', () => {
    closeSettings();
    moveMarker = true;
    map.once('click', function(e) {
            base_marker.setLatLng(e.latlng);
            let baseLat = base_marker.getLatLng().lat;
            let baseLng = base_marker.getLatLng().lng;
            socket.emit('set-base-coords', {
                lat: baseLat,
                lon: baseLng
            });
            moveMarker = false;
            
        });
})

setWinchDepthBtn.addEventListener('click', () => {
    if (maxWinchDepthInput != null) {
        socket.emit('set-max-winch-depth', maxWinchDepthInput.value);
        maxWinchDepthInput.placeholder = maxWinchDepthInput.value + "ft";
    }
})

restoreWaypointsBtn.addEventListener('click', () => {
    socket.emit("restore-waypoints");
})


sendGainsBtn.addEventListener('click', () => {
    prev_posGains = { ...posGains };
    prev_headGains = { ...headGains };
    closeSettings();
    socket.emit('set-gains', {
       heading: headGains,
       position: posGains
    });
    totalOverlay.classList.remove('hidden');
});

restoreGainsBtn.addEventListener('click', () => {
    socket.emit('restore-gains');
});

socket.on('gain_update', function(data) {
    let headGains_og = data.heading;
    let posGains_og = data.position;
    updateGains(headGains_og, posGains_og);
});




const errCode_map = {0x00: 'None ', 0x01: 'No Target', 0x02: 'No GPS', 0x10: 'Bridge Err'};
function handleIncomingErrors(error_code) {
    if (connectionStatus.textContent != "Disconnected") {
        const p = document.createElement('p');
        p.classList.add('err-tag');
        p.classList.add('glass-card');
        let time = updateClock();
        p.textContent = `${time}  |  ${errCode_map[error_code]}`;
        errsDiv.prepend(p);
    }
}


autoReturnToggle.addEventListener('click', () => {
    closeSettings();
    socket.emit('toggle-auto-return', autoReturnToggle.checked);
    totalOverlay.classList.remove('hidden');
});

webserverRestartBtn.addEventListener('click', () => {
    socket.emit('restart-webserver');
});