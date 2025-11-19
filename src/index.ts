const canvas = document.getElementById('gamePhysicsCanvas') as HTMLCanvasElement;
const ctx = canvas.getContext('2d');

if (!ctx) {
    throw new Error('Failed to get 2D context for the canvas.');
}

canvas.width = window.innerWidth;
canvas.height = window.innerHeight;
