const socket = io();

// Selection states
let eggSelected = false;
let greenSelected = false;

// Global timer variable (360 seconds = 6 minutes)
let timerInterval = null;
let currentSec = 360; 

// Button Toggle Functions
function toggleEgg() {
    eggSelected = !eggSelected;
    document.getElementById("btn_egg").classList.toggle("selected", eggSelected);
}

function toggleGreen() {
    greenSelected = !greenSelected;
    document.getElementById("btn_green").classList.toggle("selected", greenSelected);
}

// STOP (Emergency / Pause)
function pressStop() {
    socket.emit("stop_signal", true);
    console.log("STOP signal sent");
    
    pauseTimer(); 
    document.getElementById("progress_text").innerHTML = "Status: Paused";
}

// RECOVERY (System Reset)
function pressRecovery() {
    socket.emit("recovery_signal", true);
    console.log("RECOVERY signal sent - System Reset");
    
    resetSystem();
    document.getElementById("progress_text").innerHTML = "Status (0): Waiting for Order";
}

// START (Process New Order)
function pressStart() {
    let mode = 0;
    if (eggSelected && greenSelected) mode = 3;
    else if (eggSelected) mode = 1;
    else if (greenSelected) mode = 2;

    if (mode === 0) {
        alert("Please select at least one item.");
        return;
    }

    socket.emit("start_signal", { mode: mode });
    console.log(`START signal sent - Mode: ${mode}`);

    // Start timer from 6 minutes
    startNewTimer();

    // Reset selection UI
    eggSelected = false;
    greenSelected = false;
    document.getElementById("btn_egg").classList.remove("selected");
    document.getElementById("btn_green").classList.remove("selected");
}


/* ---------------- Timer Functions ---------------- */

// 1. Reset and start timer from 6:00
function startNewTimer() {
    currentSec = 360; 
    resumeTimer();
}

// 2. Core timer logic
function resumeTimer() {
    clearInterval(timerInterval);

    timerInterval = setInterval(() => {
        const m = String(Math.floor(currentSec / 60)).padStart(2, '0');
        const s = String(currentSec % 60).padStart(2, '0');

        document.getElementById("timer").innerText = `${m}:${s}`;

        if (currentSec <= 0) {
            clearInterval(timerInterval);
        }
        currentSec--; 
    }, 1000);
}

// 3. Pause timer
function pauseTimer() {
    clearInterval(timerInterval);
}

// 4. Complete system reset (For Recovery button)
function resetSystem() {
    clearInterval(timerInterval);
    currentSec = 360;
    document.getElementById("timer").innerText = "06:00";
}

/* ----------- ROS Progress Status ----------- */
socket.on("progress_update", (data) => {
    // data example: { step: 1, text: "Robot 1 Undocking..." }
    const statusText = `Status (${data.step}): ${data.text}`;
    document.getElementById("progress_text").innerHTML = statusText;
    console.log(`Progress Update: ${statusText}`);
});