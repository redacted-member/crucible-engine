// SPDX-License-Identifier: MIT
// Main thread: loads the wasm, starts the worklet, maps computer-keyboard keys to notes and draws the level meter.

const KEYS = 'awsedftgyhujkolp;'; // white and black keys of one and a half octaves, starting at C
const BASE_NOTE = 48;

type WorkletMessage =
    | { type: 'ready'; sampleRate: number; wasmBytes: number; startTime: number }
    | { type: 'failed' }
    | { type: 'meter'; peak: number; blocks: number };

const startButton = document.getElementById('start') as HTMLButtonElement;
const statusText = document.getElementById('status') as HTMLElement;
const meterBar = document.getElementById('meter-bar') as HTMLElement;
const meterValue = document.getElementById('meter-value') as HTMLElement;

let node: AudioWorkletNode | null = null;
const held = new Set<string>();

function send(status: number, note: number, velocity: number): void {
    node?.port.postMessage({ type: 'midi', status, data1: note, data2: velocity });
}

function onMeter(peak: number): void {
    const db = peak > 0 ? 20 * Math.log10(peak) : -120;
    const percent = Math.max(0, Math.min(100, ((db + 60) / 60) * 100));
    meterBar.style.setProperty('--level', `${percent}%`);
    meterValue.textContent = `${db.toFixed(1)} dBFS`;
    document.body.dataset.peak = peak.toFixed(6);
}

async function start(): Promise<void> {
    startButton.disabled = true;
    statusText.textContent = 'Loading engine...';
    const context = new AudioContext({ latencyHint: 'interactive' });
    await context.audioWorklet.addModule('engine-worklet.js');
    const bytes = await (await fetch('crucible.wasm')).arrayBuffer();
    node = new AudioWorkletNode(context, 'crucible-processor', { outputChannelCount: [2] });
    node.port.onmessage = (event: MessageEvent<WorkletMessage>) => {
        const message = event.data;
        if (message.type === 'ready') {
            statusText.textContent = `Running at ${message.sampleRate} Hz, wasm ${(message.wasmBytes / 1e6).toFixed(2)} MB. Play with the keys A to ; on the keyboard.`;
            document.body.dataset.ready = 'true';
        } else if (message.type === 'failed') {
            statusText.textContent = 'The engine failed to start.';
        } else {
            onMeter(message.peak);
        }
    };
    if (new URLSearchParams(location.search).has('mute')) {
        // Silent mode for checks: the worklet still runs because it is connected to the destination through a zero gain.
        const silence = context.createGain();
        silence.gain.value = 0;
        node.connect(silence).connect(context.destination);
    } else {
        node.connect(context.destination);
    }
    node.port.postMessage({ type: 'wasm', bytes }, [bytes]);
    await context.resume();
}

window.addEventListener('keydown', (event) => {
    const index = KEYS.indexOf(event.key.toLowerCase());
    if (index < 0 || event.repeat || event.metaKey || event.ctrlKey || held.has(event.key)) return;
    held.add(event.key);
    send(0x90, BASE_NOTE + index, 100);
});

window.addEventListener('keyup', (event) => {
    const index = KEYS.indexOf(event.key.toLowerCase());
    if (index < 0 || !held.delete(event.key)) return;
    send(0x80, BASE_NOTE + index, 0);
});

startButton.addEventListener('click', () => {
    start().catch((error: unknown) => {
        statusText.textContent = `Error: ${String(error)}`;
        startButton.disabled = false;
    });
});

// Test hook for the page check: plays a note without a keyboard.
(window as unknown as { playTestNote: () => void }).playTestNote = () => send(0x90, 60, 100);
