export class Road {
    constructor(id, name, startPoint, endPoint, width = 60) {
        this.id = id;
        this.name = name;
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        this.width = width;
    }

    /**
     * Vẽ đường thẳng lên Canvas
     * @param {CanvasRenderingContext2D} ctx 
     */
    draw(ctx) {
        // Thiết lập chung cho đường đi
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';

        // 1. Vẽ thân đường (màu xám tối)
        ctx.beginPath();
        ctx.moveTo(this.startPoint.x, this.startPoint.y);
        ctx.lineTo(this.endPoint.x, this.endPoint.y);
        
        ctx.lineWidth = this.width;
        ctx.strokeStyle = '#4A4A4A';
        ctx.stroke();

        // 2. Vẽ vạch kẻ đường ở giữa (màu trắng, đứt đoạn)
        ctx.beginPath();
        ctx.moveTo(this.startPoint.x, this.startPoint.y);
        ctx.lineTo(this.endPoint.x, this.endPoint.y);
        
        ctx.lineWidth = Math.max(2, this.width / 20);
        ctx.strokeStyle = '#FFFFFF';
        ctx.setLineDash([20, 20]); // Tạo nét đứt
        ctx.stroke();
        
        // Reset nét đứt để không ảnh hưởng các đối tượng khác
        ctx.setLineDash([]); 
    }
}

export class Building {
    constructor(id, name, height, width, type, position, link) {
        this.id = id;
        this.name = name;
        this.height = height; // Pixel
        this.width = width;   // Pixel
        this.type = type;
        this.position = position; // {x, y}
        this.link = link;
        
        // Tạo đối tượng ảnh để cache
        this.img = new Image();
        this.img.src = this.link;
        this.loaded = false;
        this.img.onload = () => { this.loaded = true; };
    }

    /**
     * Vẽ tòa nhà lên Canvas
     * @param {CanvasRenderingContext2D} ctx 
     */
    draw(ctx) {
        if (!this.loaded) return; // Đợi ảnh tải xong mới vẽ

        // Vẽ ảnh tại vị trí (x, y) với kích thước (width, height)
        // Chúng ta trừ đi một nửa width/height để position nằm chính giữa tâm tòa nhà
        const drawX = this.position.x - this.width / 2;
        const drawY = this.position.y - this.height / 2;

        ctx.drawImage(this.img, drawX, drawY, this.width, this.height);

        // Tùy chọn: Vẽ thêm tên tòa nhà bên dưới
        ctx.fillStyle = '#FFFFFF';
        ctx.font = '12px Arial';
        ctx.textAlign = 'center';
        ctx.fillText(this.name, this.position.x, drawY + this.height + 15);
    }
}

/**
 * Lớp đại diện cho giao lộ tiêu chuẩn (Ngã 3, ngã 4)
 */
export class Intersection {
    constructor(id, name, position, numberOfRoads, angle = 0, width = 50) {
        this.id = id;
        this.name = name;
        this.position = position; // {x, y}
        this.numberOfRoads = numberOfRoads; // 3 hoặc 4
        this.angle = angle; // Độ nghiêng (xoay theo trục tâm)
        this.width = width; // Độ rộng của lòng đường tại giao lộ
        this.type = 'standard';
    }
    
    draw(ctx) {
        ctx.strokeStyle = '#4A4A4A'; // Màu xám cho đường
        ctx.lineWidth = this.width;
        ctx.lineCap = 'round';

        const baseAngle = this.angle * (Math.PI / 180);
        const spacing = (2 * Math.PI) / this.numberOfRoads;

        for (let i = 0; i < this.numberOfRoads; i++) {
            const angle = baseAngle + (i * spacing);
            // Độ dài nhánh tỏa ra từ tâm (ví dụ: width * 1.5)
            const length = this.width * 1.5; 
            
            ctx.beginPath();
            ctx.moveTo(this.position.x, this.position.y);
            ctx.lineTo(
                this.position.x + Math.cos(angle) * length,
                this.position.y + Math.sin(angle) * length
            );
            ctx.stroke();
        }
    }
}

/**
 * Lớp đại diện cho vòng xoay (Roundabout)
 */
export class Roundabout extends Intersection {
    constructor(id, name, position, numberOfRoads, innerRadius, outerRadius) {
        // Roundabout không cần angle vì nó đối xứng hình tròn
        super(id, name, position, numberOfRoads, 0, (outerRadius * 2));
        this.innerRadius = innerRadius;
        this.outerRadius = outerRadius;
        this.type = 'roundabout';
    }
    
    draw(ctx) {
        // 1. Vẽ mặt đường vòng xoay (vành ngoài)
        ctx.beginPath();
        ctx.arc(this.position.x, this.position.y, this.outerRadius, 0, Math.PI * 2);
        ctx.fillStyle = '#4A4A4A'; // Màu xám mặt đường
        ctx.fill();

        // 2. Vẽ đảo ở giữa (để hở phần đường xám)
        ctx.beginPath();
        ctx.arc(this.position.x, this.position.y, this.innerRadius, 0, Math.PI * 2);
        ctx.fillStyle = '#1a1a1a'; // Màu nền của bản đồ (để tạo cảm giác rỗng ở giữa)
        ctx.fill();

        // 3. (Tùy chọn) Vẽ viền cho vòng xoay
        ctx.strokeStyle = '#666';
        ctx.lineWidth = 2;
        ctx.stroke();
    }
}

export class TrafficLight {
    constructor(id, link, position, cycleTime, curTime, color = "green") {
        this.id = id;
        this.link = link; // Ví dụ: "../assets/sprites/light.png"
        this.position = position; // {x, y}
        this.cycleTime = cycleTime;
        this.curTime = curTime;
        this.color = color; // "green", "yellow", "red"
        
        // Cấu hình kích thước cố định
        this.width = 30;
        this.height = 30;

        // Xử lý link ảnh dựa trên màu
        this.image = new Image();
        this.updateImageLink();
    }

    // Hàm tạo link ảnh động dựa trên màu
    updateImageLink() {
        const lastDotIndex = this.link.lastIndexOf('.');
        const path = this.link.substring(0, lastDotIndex);
        const ext = this.link.substring(lastDotIndex);
        this.image.src = `${path}-${this.color}${ext}`;
    }

    /**
     * @param {CanvasRenderingContext2D} ctx 
     */
    draw(ctx) {
        const drawX = this.position.x - this.width / 2;
        const drawY = this.position.y - this.height / 2;

        // Vẽ đèn giao thông
        ctx.drawImage(this.image, drawX, drawY, this.width, this.height);
    }
}

export class Decoration {
    constructor(id, position, name, link) {
        this.id = id;
        this.position = position; // {x, y}
        this.name = name;
        this.link = link;
        
        // Thiết lập kích thước mặc định cho Decoration
        this.width = 75; 
        this.height = 75;
        
        // Tạo đối tượng ảnh để cache
        this.img = new Image();
        this.img.src = this.link;
        this.loaded = false;
        this.img.onload = () => { this.loaded = true; };
    }
    
    /**
     * Vẽ vật thể trang trí lên Canvas
     * @param {CanvasRenderingContext2D} ctx 
     */
    draw(ctx) {
        if (!this.loaded) return; // Đợi ảnh tải xong mới vẽ

        // Vẽ ảnh tại vị trí (x, y) với kích thước (width, height)
        // Trừ đi một nửa để position nằm chính giữa tâm vật thể
        const drawX = this.position.x - this.width / 2;
        const drawY = this.position.y - this.height / 2;

        ctx.drawImage(this.img, drawX, drawY, this.width, this.height);    }
}

export class Vehicle {
    constructor(id, position, angle, link) {
        this.id = id;
        this.position = position; // {x, y}
        this.angle = angle;       // Góc xoay bằng độ (0 là bên phải)
        this.link = link;
        
        // Kích thước cố định cho xe
        this.width = 40;
        this.height = 20;

        // Load ảnh
        this.image = new Image();
        this.image.src = this.link;
    }

    /**
     * @param {CanvasRenderingContext2D} ctx 
     */
    draw(ctx) {
        // Lưu trạng thái cũ của context để không ảnh hưởng các vật thể khác
        ctx.save();

        // 1. Dời tâm xoay về đúng vị trí xe
        ctx.translate(this.position.x, this.position.y);

        // 2. Xoay canvas theo góc của xe
        // Chuyển độ sang radian: góc * (PI / 180)
        ctx.rotate(this.angle * (Math.PI / 180));

        // 3. Vẽ xe tại vị trí trung tâm (đã dịch chuyển)
        // Dùng -width/2 và -height/2 để xe nằm giữa tâm xoay
        ctx.drawImage(
            this.image, 
            -this.width / 2, 
            -this.height / 2, 
            this.width, 
            this.height
        );

        // Phục hồi trạng thái cũ
        ctx.restore();
    }
}

export class World {
    constructor() {
        this.objects = [];
    }
    addObject(obj) { this.objects.push(obj); }
    // Thứ tự vẽ: Đường -> Giao lộ -> Tòa nhà/Trang trí -> Xe/Đèn (để tránh bị đè)
    draw(ctx) {
        // Sắp xếp các đối tượng theo loại để vẽ đúng layer
        const layerOrder = ['Road', 'Intersection', 'Roundabout', 'Building', 'Decoration', 'TrafficLight', 'Vehicle'];
        
        // Vẽ từng layer theo thứ tự
        layerOrder.forEach(type => {
        this.objects.forEach(obj => {
            if (obj.constructor.name === type && typeof obj.draw === 'function') {
                try {
                    // Dùng try-catch để nếu 1 cái cây bị lỗi, 
                    // nó chỉ bỏ qua cái cây đó chứ không làm hỏng cả World
                    obj.draw(ctx);
                } catch (e) {
                    console.error("Lỗi khi vẽ đối tượng:", obj, e);
                }
            }
        });
    });
    }
}