export class Camera {
    constructor(canvas) {
        this.canvas = canvas;
        this.x = window.innerWidth / 2;
        this.y = window.innerHeight / 2;
        this.zoom = 1;
        this.isDragging = false;
        this.dirty = true;   // set whenever the view changes; the app redraws on it

        this.initEvents();
    }
    
    initEvents() {
        this.canvas.addEventListener('mousedown', (e) => {
            // Chỉ bắt đầu kéo nếu click TRỰC TIẾP vào canvas
            if(e.button === 0 && e.target === this.canvas) {
                this.isDragging = true;
            }
        });
        
        window.addEventListener('mouseup', () => {
            this.isDragging = false;
        });
        
        window.addEventListener('mousemove', (e) => {
            if (this.isDragging) {
                this.x += e.movementX;
                this.y += e.movementY;
                this.dirty = true;
            }
        });
        
        this.canvas.addEventListener('wheel', (e) => {
            if(e.target !== this.canvas) return; // Bỏ qua nếu lăn chuột trên log/sidebar
            
            const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
            const newZoom = this.zoom * zoomFactor;
            
            if (newZoom < 0.1 || newZoom > 5) return;
            
            const mouseX = e.clientX;
            const mouseY = e.clientY;
            
            this.x = mouseX - (mouseX - this.x) * (newZoom / this.zoom);
            this.y = mouseY - (mouseY - this.y) * (newZoom / this.zoom);
            this.zoom = newZoom;
            this.dirty = true;
        }, { passive: true });
    }
    
    begin(ctx) {
        ctx.save();
        ctx.translate(this.x, this.y);
        ctx.scale(this.zoom, this.zoom);
    }
    
    end(ctx) {
        ctx.restore();
    }
}