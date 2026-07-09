let isSimulationPaused = false; // Biến kiểm soát trạng thái

export function animate(canvas, ctx, camera, world) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    camera.begin(ctx);
    
    // Logic của thế giới
    if (!isSimulationPaused) {
        //world.update(); // Chỉ cho phép xe di chuyển khi không bị pause
    }
    world.draw(ctx);
    
    camera.end(ctx);
    requestAnimationFrame(animate);
}