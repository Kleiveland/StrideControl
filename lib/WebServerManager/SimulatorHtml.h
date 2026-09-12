#pragma once

#if defined(STRIDECONTROL_TESTBENCH)
#include <Arduino.h>

namespace stridecontrol {

static const char kSimulatorHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>StrideControl — Physical Universe Simulator</title>
    <style>
        :root {
            --bg: #0d1117;
            --surface: #161b22;
            --surface-border: #30363d;
            --text: #c9d1d9;
            --text-dim: #8b949e;
            --accent: #58a6ff;
            --green: #2ea043;
            --green-hover: #3fb950;
            --red: #da3633;
            --red-hover: #f85149;
            --card-bg: #21262d;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif; }
        body { background-color: var(--bg); color: var(--text); padding: 20px; line-height: 1.5; }
        .container { max-width: 1080px; margin: 0 auto; }
        header { display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid var(--surface-border); padding-bottom: 15px; margin-bottom: 20px; }
        h1 { font-size: 1.4rem; font-weight: 600; color: #fff; }
        .badge { display: inline-block; padding: 2px 8px; font-size: 0.75rem; border-radius: 12px; font-weight: 600; text-transform: uppercase; }
        .badge-live { background-color: rgba(46, 160, 67, 0.2); color: #3fb950; border: 1px solid #2ea043; }
        .nav-links a { color: var(--accent); text-decoration: none; font-size: 0.9rem; margin-left: 15px; }
        .nav-links a:hover { text-decoration: underline; }

        /* Telemetry Diagnostic Strip */
        .telemetry-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 12px; margin-bottom: 24px; }
        .telem-card { background: var(--surface); border: 1px solid var(--surface-border); border-radius: 8px; padding: 12px; text-align: center; }
        .telem-label { font-size: 0.75rem; color: var(--text-dim); text-transform: uppercase; margin-bottom: 4px; letter-spacing: 0.5px; }
        .telem-val { font-size: 1.3rem; font-weight: 700; color: #fff; font-family: "SFMono-Regular", Consolas, monospace; }
        .telem-sub { font-size: 0.75rem; color: var(--text-dim); margin-top: 4px; }

        /* Simulator Panels Grid */
        .panels-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-bottom: 20px; }
        @media (max-width: 768px) { .panels-grid { grid-template-columns: 1fr; } }

        .panel { background: var(--surface); border: 1px solid var(--surface-border); border-radius: 8px; padding: 18px; }
        .panel-title { font-size: 1.05rem; font-weight: 600; color: #fff; margin-bottom: 15px; border-bottom: 1px solid var(--surface-border); padding-bottom: 8px; display: flex; justify-content: space-between; align-items: center; }
        
        /* Buttons & Controls */
        .btn-row { display: flex; gap: 10px; margin-bottom: 14px; flex-wrap: wrap; }
        button { cursor: pointer; border: none; border-radius: 6px; font-weight: 600; font-size: 0.9rem; padding: 10px 16px; transition: all 0.15s ease; color: #fff; }
        button:active { transform: scale(0.98); }
        .btn-green { background-color: var(--green); }
        .btn-green:hover { background-color: var(--green-hover); }
        .btn-red { background-color: var(--red); }
        .btn-red:hover { background-color: var(--red-hover); }
        .btn-dark { background-color: var(--card-bg); border: 1px solid var(--surface-border); }
        .btn-dark:hover { background-color: #30363d; border-color: #8b949e; }
        .btn-step { min-width: 90px; }
        .btn-estop { background-color: #b62324; width: 100%; padding: 14px; font-size: 1rem; letter-spacing: 1px; text-transform: uppercase; margin-top: 6px; }
        .btn-estop:hover { background-color: #d9383a; }

        .control-group { margin-bottom: 16px; }
        .group-label { font-size: 0.85rem; color: var(--text-dim); margin-bottom: 6px; font-weight: 600; }
        .input-row { display: flex; gap: 8px; }
        input[type="number"] { background: var(--card-bg); border: 1px solid var(--surface-border); border-radius: 6px; color: #fff; padding: 8px 12px; font-size: 0.95rem; width: 100px; font-family: monospace; }
        input[type="number"]:focus { border-color: var(--accent); outline: none; }

        /* Runner Controls */
        .radio-group { display: flex; flex-direction: column; gap: 10px; margin-bottom: 18px; }
        .radio-option { display: flex; align-items: center; gap: 10px; background: var(--card-bg); padding: 10px 14px; border-radius: 6px; border: 1px solid var(--surface-border); cursor: pointer; }
        .radio-option:hover { border-color: #8b949e; }
        .radio-option input[type="radio"] { cursor: pointer; accent-color: var(--accent); transform: scale(1.2); }
        .slider-group { margin-bottom: 18px; }
        .slider-header { display: flex; justify-content: space-between; margin-bottom: 6px; font-size: 0.85rem; font-weight: 600; }
        input[type="range"] { width: 100%; accent-color: var(--accent); cursor: pointer; }
        .checkbox-group { display: flex; align-items: center; gap: 10px; background: var(--card-bg); padding: 10px 14px; border-radius: 6px; border: 1px solid var(--surface-border); cursor: pointer; }
        .checkbox-group input[type="checkbox"] { cursor: pointer; accent-color: var(--green); transform: scale(1.2); }

        /* Status Bar */
        .status-bar { font-size: 0.8rem; color: var(--text-dim); display: flex; justify-content: space-between; background: var(--surface); border: 1px solid var(--surface-border); border-radius: 6px; padding: 8px 14px; }
        .status-success { color: #3fb950; }
        .status-error { color: #f85149; }
    </style>
</head>
<body>
<div class="container">
    <header>
        <div>
            <h1>T610 Physical Treadmill & Runner Simulator</h1>
            <span style="font-size: 0.8rem; color: var(--text-dim);">Firmware Testbench Active Object &bull; External Universe</span>
        </div>
        <div class="nav-links">
            <span class="badge badge-live" id="conn-badge">LIVE @ 250ms</span>
            <a href="/index.html" target="_blank">&rarr; Open StrideControl Product GUI</a>
        </div>
    </header>

    <!-- REAL-TIME TELEMETRY DIAGNOSTIC STRIP -->
    <div class="telemetry-grid">
        <div class="telem-card">
            <div class="telem-label">Belt Speed vs Runner</div>
            <div class="telem-val" id="t-speed">0.00 <span style="font-size:0.8rem">km/h</span></div>
            <div class="telem-sub">Runner: <span id="t-runner-speed">0.00</span> km/h</div>
        </div>
        <div class="telem-card">
            <div class="telem-label">Odometer vs Validated</div>
            <div class="telem-val" id="t-dist">0.000 <span style="font-size:0.8rem">km</span></div>
            <div class="telem-sub">Runner: <span id="t-runner-dist">0.000</span> km</div>
        </div>
        <div class="telem-card">
            <div class="telem-label">Incline (Actual / Target)</div>
            <div class="telem-val" id="t-incline">0.0 <span style="font-size:0.8rem">%</span></div>
            <div class="telem-sub">Target: <span id="t-target-incline">0.0</span> %</div>
        </div>
        <div class="telem-card">
            <div class="telem-label">Runner Presence</div>
            <div class="telem-val" style="font-size: 1.1rem; color: var(--accent)" id="t-presence">UNKNOWN</div>
            <div class="telem-sub">Cadence: <span id="t-cadence">--</span> spm</div>
        </div>
        <div class="telem-card">
            <div class="telem-label">Workout Session</div>
            <div class="telem-val" style="font-size: 1.1rem; color: var(--green-hover)" id="t-session">Idle</div>
            <div class="telem-sub">Step Rem: <span id="t-step-rem">--</span>s</div>
        </div>
        <div class="telem-card">
            <div class="telem-label">Simulator Health</div>
            <div class="telem-val" style="font-size: 1.1rem;" id="t-dropped">0 <span style="font-size:0.8rem">dropped</span></div>
            <div class="telem-sub">Auth: <span id="t-authority" style="color:var(--green-hover)">YES</span></div>
        </div>
    </div>

    <!-- MAIN CONTROLS GRID -->
    <div class="panels-grid">
        <!-- PANEL A: PHYSICAL T610 CONSOLE EMULATION -->
        <div class="panel">
            <div class="panel-title">
                <span>T610 Physical Console Emulation</span>
                <span style="font-size:0.75rem; color:var(--text-dim)">POST /api/v1/simulator/console</span>
            </div>

            <!-- Major Action Buttons -->
            <div class="control-group">
                <div class="group-label">Primary Actions</div>
                <div class="btn-row">
                    <button class="btn-green" style="flex:1" onclick="sendConsole('QuickStart')">QUICK START (1.0 km/h)</button>
                    <button class="btn-red" style="flex:1" onclick="sendConsole('Stop')">STOP (0.0 km/h)</button>
                </div>
                <button class="btn-estop" onclick="sendConsole('EmergencyStop')">EMERGENCY STOP (Latch 0.0 km/h)</button>
            </div>

            <!-- Speed Increments -->
            <div class="control-group">
                <div class="group-label">Speed Stepping (&plusmn;0.1 km/h) & Presets</div>
                <div class="btn-row">
                    <button class="btn-dark btn-step" onclick="sendConsole('SpeedMinus')">&minus; 0.1 km/h</button>
                    <button class="btn-dark btn-step" onclick="sendConsole('SpeedPlus')">&plus; 0.1 km/h</button>
                    <button class="btn-dark" onclick="sendConsole('SetSpeed', 3.0)">3.0</button>
                    <button class="btn-dark" onclick="sendConsole('SetSpeed', 6.0)">6.0</button>
                    <button class="btn-dark" onclick="sendConsole('SetSpeed', 10.0)">10.0</button>
                    <button class="btn-dark" onclick="sendConsole('SetSpeed', 12.0)">12.0</button>
                </div>
                <div class="input-row">
                    <input type="number" id="num-speed" min="0.8" max="22.0" step="0.1" value="10.0">
                    <button class="btn-dark" onclick="sendConsole('SetSpeed', parseFloat(document.getElementById('num-speed').value))">Set Speed Target</button>
                </div>
            </div>

            <!-- Incline Increments (0.5% contract verified in production code) -->
            <div class="control-group">
                <div class="group-label">Incline Stepping (&plusmn;0.5 % Production Step) & Presets</div>
                <div class="btn-row">
                    <button class="btn-dark btn-step" onclick="sendConsole('InclineMinus')">&minus; 0.5 %</button>
                    <button class="btn-dark btn-step" onclick="sendConsole('InclinePlus')">&plus; 0.5 %</button>
                    <button class="btn-dark" onclick="sendConsole('SetIncline', 0.0)">0.0%</button>
                    <button class="btn-dark" onclick="sendConsole('SetIncline', 1.0)">1.0%</button>
                    <button class="btn-dark" onclick="sendConsole('SetIncline', 2.0)">2.0%</button>
                    <button class="btn-dark" onclick="sendConsole('SetIncline', 3.0)">3.0%</button>
                    <button class="btn-dark" onclick="sendConsole('SetIncline', 5.0)">5.0%</button>
                </div>
                <div class="input-row">
                    <input type="number" id="num-incline" min="0.0" max="15.0" step="0.5" value="3.0">
                    <button class="btn-dark" onclick="sendConsole('SetIncline', parseFloat(document.getElementById('num-incline').value))">Set Incline Target</button>
                </div>
            </div>
        </div>

        <!-- PANEL B: VIRTUAL RUNNER DYNAMICS -->
        <div class="panel">
            <div class="panel-title">
                <span>Virtual Runner Dynamics</span>
                <span style="font-size:0.75rem; color:var(--text-dim)">POST /api/v1/simulator/runner</span>
            </div>

            <!-- Runner Location -->
            <div class="control-group">
                <div class="group-label">Runner Position</div>
                <div class="radio-group">
                    <label class="radio-option">
                        <input type="radio" name="runner-loc" value="RunningOnBelt" checked onchange="sendRunner()">
                        <div>
                            <strong>Running On Belt</strong>
                            <div style="font-size:0.75rem; color:var(--text-dim)">Normal footstrikes, full distance accumulation & speed credit</div>
                        </div>
                    </label>
                    <label class="radio-option">
                        <input type="radio" name="runner-loc" value="OnSideRails" onchange="sendRunner()">
                        <div>
                            <strong>On Side Rails</strong>
                            <div style="font-size:0.75rem; color:var(--text-dim)">No footstrikes: triggers 750ms grace period &rarr; distance accumulation freeze</div>
                        </div>
                    </label>
                    <label class="radio-option">
                        <input type="radio" name="runner-loc" value="NotPresent" onchange="sendRunner()">
                        <div>
                            <strong>Not Present</strong>
                            <div style="font-size:0.75rem; color:var(--text-dim)">Empty treadmill: no runner signal, automatic pause evaluation</div>
                        </div>
                    </label>
                </div>
            </div>

            <!-- Cadence Slider -->
            <div class="slider-group">
                <div class="slider-header">
                    <span>Cadence</span>
                    <span id="cadence-display" style="color:var(--accent); font-family:monospace;">180 SPM</span>
                </div>
                <input type="range" id="cadence-slider" min="120" max="200" step="2" value="180" oninput="onCadenceChange(this.value)">
            </div>

            <!-- IMU Validity Checkbox -->
            <div class="control-group">
                <label class="checkbox-group">
                    <input type="checkbox" id="imu-valid" checked onchange="sendRunner()">
                    <div>
                        <strong>IMU Hardware Signal Valid</strong>
                        <div style="font-size:0.75rem; color:var(--text-dim)">Simulates healthy accelerometer/gyroscope I2C telemetry stream</div>
                    </div>
                </label>
            </div>
        </div>

        <!-- PANEL C: SIMULATED HEART RATE -->
        <div class="panel">
            <div class="panel-title">
                <span>Simulated Heart Rate</span>
                <span style="font-size:0.75rem; color:var(--text-dim)">POST /api/v1/simulator/heartrate</span>
            </div>
            <div class="control-group">
                <label class="checkbox-group">
                    <input type="checkbox" id="sim-hr-speed" onchange="sendHeartRateSim()">
                    <div>
                        <strong>Simulate HR from Belt Speed</strong>
                        <div style="font-size:0.75rem; color:var(--text-dim)">Derives BPM from current belt speed (80 + (speed &minus; 1.0) &times; 6.36, clamped [70, 200]). Overrides BLE strap when active.</div>
                    </div>
                </label>
            </div>
        </div>
    </div>

    <!-- FOOTER / STATUS BAR -->
    <div class="status-bar">
        <span id="status-msg">Simulator ready. Polling telemetry...</span>
        <span id="status-ts">--</span>
    </div>
</div>

<script>
    async function sendConsole(button, value = 0.0) {
        setStatus(`Sending console intent: ${button}...`);
        try {
            const body = { button: button };
            if (button === 'SetSpeed' || button === 'SetIncline') {
                body.value = value;
            }
            const res = await fetch('/api/v1/simulator/console', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(body)
            });
            if (res.ok) {
                setStatus(`Console intent staged: ${button} ${value > 0 ? value : ''}`, 'success');
            } else {
                const err = await res.json();
                setStatus(`Failed: ${err.error || res.statusText}`, 'error');
            }
        } catch (e) {
            setStatus(`Network error sending console intent: ${e.message}`, 'error');
        }
    }

    function onCadenceChange(val) {
        document.getElementById('cadence-display').innerText = `${val} SPM`;
        sendRunner();
    }

    async function sendRunner() {
        const mode = document.querySelector('input[name="runner-loc"]:checked').value;
        const cadence = parseInt(document.getElementById('cadence-slider').value, 10);
        const imuValid = document.getElementById('imu-valid').checked;

        setStatus(`Updating runner dynamics: ${mode} @ ${cadence} SPM...`);
        try {
            const res = await fetch('/api/v1/simulator/runner', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    mode: mode,
                    cadence: cadence,
                    impactMagnitudeG: 0.35,
                    signalValid: imuValid
                })
            });
            if (res.ok) {
                setStatus(`Runner state staged: ${mode} (${cadence} SPM)`, 'success');
            } else {
                const err = await res.json();
                setStatus(`Runner update failed: ${err.error || res.statusText}`, 'error');
            }
        } catch (e) {
            setStatus(`Network error updating runner: ${e.message}`, 'error');
        }
    }

    async function sendHeartRateSim() {
        const enabled = document.getElementById('sim-hr-speed').checked;
        setStatus(`Setting simulated heart rate override: ${enabled ? 'ENABLED' : 'DISABLED'}...`);
        try {
            const res = await fetch('/api/v1/simulator/heartrate', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ simulateFromSpeed: enabled })
            });
            if (res.ok) {
                setStatus(`Simulated heart rate ${enabled ? 'enabled' : 'disabled'}`, 'success');
            } else {
                const err = await res.json();
                setStatus(`HR simulation update failed: ${err.error || res.statusText}`, 'error');
            }
        } catch (e) {
            setStatus(`Network error updating HR simulation: ${e.message}`, 'error');
        }
    }

    function setStatus(msg, type = '') {
        const el = document.getElementById('status-msg');
        el.innerText = msg;
        el.className = type === 'success' ? 'status-success' : (type === 'error' ? 'status-error' : '');
        document.getElementById('status-ts').innerText = new Date().toLocaleTimeString();
    }

    async function pollTelemetry() {
        try {
            const res = await fetch('/api/v1/telemetry');
            if (res.ok) {
                const data = await res.json();
                updateTelemetryUI(data);
                document.getElementById('conn-badge').className = 'badge badge-live';
                document.getElementById('conn-badge').innerText = 'LIVE @ 250ms';
            } else {
                document.getElementById('conn-badge').className = 'badge';
                document.getElementById('conn-badge').style.background = '#da3633';
                document.getElementById('conn-badge').innerText = 'ERROR';
            }
        } catch (e) {
            document.getElementById('conn-badge').className = 'badge';
            document.getElementById('conn-badge').style.background = '#6e7681';
            document.getElementById('conn-badge').innerText = 'DISCONNECTED';
        }
    }

    function updateTelemetryUI(data) {
        if (data.speed) {
            const beltKmh = typeof data.speed.kmh === 'number' ? data.speed.kmh.toFixed(2) : '--';
            document.getElementById('t-speed').innerHTML = `${beltKmh} <span style="font-size:0.8rem">km/h</span>`;
            if (typeof data.speed.beltDistanceKm === 'number') {
                document.getElementById('t-dist').innerHTML = `${data.speed.beltDistanceKm.toFixed(3)} <span style="font-size:0.8rem">km</span>`;
            }
        }
        if (data.runner) {
            const rSpeed = typeof data.runner.speedKmh === 'number' ? data.runner.speedKmh.toFixed(2) : '--';
            const rDist = typeof data.runner.distanceKm === 'number' ? data.runner.distanceKm.toFixed(3) : '--';
            document.getElementById('t-runner-speed').innerText = rSpeed;
            document.getElementById('t-runner-dist').innerText = rDist;
            if (data.runner.presence) {
                document.getElementById('t-presence').innerText = data.runner.presence;
            }
        }
        if (data.incline) {
            const incPct = typeof data.incline.pct === 'number' ? data.incline.pct.toFixed(1) : '--';
            document.getElementById('t-incline').innerHTML = `${incPct} <span style="font-size:0.8rem">%</span>`;
        }
        if (typeof data.targetInclinePct === 'number') {
            document.getElementById('t-target-incline').innerText = data.targetInclinePct.toFixed(1);
        }
        if (data.session) {
            document.getElementById('t-session').innerText = data.session.state || 'Idle';
            if (typeof data.session.stepRemainingMs === 'number') {
                document.getElementById('t-step-rem').innerText = Math.round(data.session.stepRemainingMs / 1000);
            }
        }
        if (typeof data.droppedEvents === 'number') {
            document.getElementById('t-dropped').innerHTML = `${data.droppedEvents} <span style="font-size:0.8rem">dropped</span>`;
        }
        if (typeof data.authority === 'boolean') {
            const authEl = document.getElementById('t-authority');
            authEl.innerText = data.authority ? 'YES' : 'NO';
            authEl.style.color = data.authority ? '#3fb950' : '#f85149';
        }
    }

    setInterval(pollTelemetry, 250);
    pollTelemetry();
</script>
</body>
</html>

)rawliteral";

} // namespace stridecontrol
#endif
