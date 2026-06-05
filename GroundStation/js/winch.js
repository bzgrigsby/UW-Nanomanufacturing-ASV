// Winch Box Elements
const winchSettingsIcon = document.getElementById('winch-settings-icon');
const winchSettingsCloseBtn = document.getElementById('winch-settings-close-btn');
const winchSettingsMenu = document.getElementById('winch-settings-menu');
const depthText = document.getElementById('depth-val');
const upBtn = document.getElementById("winch-up");
const dwnBtn = document.getElementById("winch-down");
const winchStopBtn = document.getElementById("winch-stop");


const A_input = document.getElementById("winch-A-input");
const T_input = document.getElementById("winch-T-input");
const O_input = document.getElementById("winch-O-input");
const timescaleTxt = document.getElementById("timescale-txt")
const winchSendBtn = document.getElementById('winch-settings-send');
const winchStopBtn2 = document.getElementById('winch-settings-stop');

const line1 = new TimeSeries();
const ghostLine = new TimeSeries();

let A_val = 5.0;
let T_val = 100.0;
let O_val = 0.0;


function updateFunctionText() {
    A_input.value = `${A_val}`;
    T_input.value = `${T_val}`;
    O_input.value = `${O_val}`;
    updateWinchChart();
}


A_input.addEventListener("change", (e) => {
    A_val = Number(e.target.value);
    ghostLine.data = [line1.data];
    updateFunctionText();
});

T_input.addEventListener("change", (e) => {
    T_val = Number(e.target.value);
    ghostLine.data = [line1.data];
    updateFunctionText();
});

O_input.addEventListener("change", (e) => {
    O_val = Number(e.target.value);
    ghostLine.data = [line1.data];
    updateFunctionText();
});


   // Winch Commands \\
upBtn.addEventListener('mousedown', function(e) {
    sendWinchCommand('up');
    dwnBtn.style.color = "black";
});
upBtn.addEventListener('mouseup', () => {
    upBtn.classList.remove("active");
});
dwnBtn.addEventListener('mousedown', function(e) {
    sendWinchCommand('down');
    upBtn.style.color = "black";
});
dwnBtn.addEventListener("mouseup", () => {
    dwnBtn.classList.remove("active");
});
winchStopBtn.addEventListener('mousedown', function(e) {
    sendWinchCommand('stop');
    upBtn.style.color = "black";
    dwnBtn.style.color = "black";
});
winchStopBtn.addEventListener('mouseup', () => {
    winchStopBtn.classList.remove("active");
});

upBtn.addEventListener('contextmenu', function(e) {
    e.preventDefault();
});
dwnBtn.addEventListener('contextmenu', function(e) {
    e.preventDefault();
});



winchSettingsIcon.addEventListener('click', () => {
    winchSettingsMenu.classList.toggle('active');
    ghostLine.data = [];
    updateFunctionText();
    
});

winchSettingsCloseBtn.addEventListener('click', ()=> {
    winchSettingsMenu.classList.toggle('active');
})

winchSendBtn.addEventListener('click', () => {
    totalOverlay.classList.remove('hidden');
    socket.emit('set-winch-auto', [A_val, T_val, O_val]);
});

winchStopBtn2.addEventListener('click', () => {
    sendWinchCommand('stop');
    upBtn.style.color = "black";
    dwnBtn.style.color = "black";
});



const winch_chart_canvas = document.getElementById('winch-graph');
winch_chart_canvas.width = winch_chart_canvas.clientWidth;
winch_chart_canvas.height = winch_chart_canvas.clientHeight;


const winch_chart = new SmoothieChart({
    grid: { 
        strokeStyle: 'rgb(90, 90, 90)', 
        fillStyle: 'rgba(28, 28, 28, 0.15)', 
        lineWidth: 1, 
        millisPerLine: 250, 
        verticalSections: 6,
        labelPadding: 10
    },
    labels: { 
        fillStyle: 'rgb(255, 255, 255)',
        fontSize: 19,
        precision: 1,
        showDataMaxValue: true 
        
    },
    interpolation: 'bezier', // Makes the movement profile look smooth
    minValue: 0,
    maxValue: 10
});

winch_chart.addTimeSeries(line1, { strokeStyle: 'rgb(221, 0, 250)', lineWidth: 3 });
winch_chart.addTimeSeries(ghostLine, { strokeStyle: 'rgba(221, 0, 250, 0.37)', lineWidth: 2 });
winch_chart.streamTo(winch_chart_canvas, 100);

function updateWinchChart() {
    const now = new Date().getTime();
    // Clear the previous line
    line1.data = [];
    for (let t = 0; t <= 100; t += 0.1) {
        let T_loop = T_val;
        if (T_val > 10) { 
            T_loop = T_val/10;
            timescaleTxt.textContent = "x10";
        } else {
            timescaleTxt.textContent = "x1";
        }
        const val = A_val*Math.abs(Math.sin(Math.PI/T_loop*t)) + O_val/100
        line1.append(now + (t * 1000), val);
    }

}
