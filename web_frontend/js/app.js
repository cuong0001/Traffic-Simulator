import { openSidebar, closeSidebar } from './menu.js';

window.openSidebar = openSidebar;
window.closeSidebar = closeSidebar;

import { prepareForm, handleAdd } from './menu.js';

window.prepareForm = prepareForm;
window.handleAdd = handleAdd;

import { add } from './features.js';

window.add = add;

import { World, Road, Building, Intersection, Vehicle, TrafficLight, Decoration } from './objects.js';
import { Camera } from './camera.js';

const canvas = document.getElementById('mainCanvas');
const ctx = canvas.getContext('2d');

const world = new World();
const camera = new Camera(canvas);

// Cấu hình kích thước canvas
function resize() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
}
window.addEventListener('resize', resize);
resize();

// Khởi tạo đối tượng mẫu
function init() {
// 1. Vẽ các con đường (Road)
    world.addObject(new Road(1, "Đường Chính", {x: -400, y: 0}, {x: 400, y: 0}, 70));
    world.addObject(new Road(2, "Đường Phụ", {x: 0, y: -400}, {x: 0, y: 400}, 70));
    world.addObject(new Intersection(3, "Trung tâm", {x: 0, y: 0}, 4, 0, 70));

    // 2. Tòa nhà (Sử dụng building-1, 2, 3 và hospital, firestation, policestation)
    const buildings = [
        { name: "City Hall", pos: {x: -150, y: -150}, link: '../assets/sprites/building-1.png' },
        { name: "Hospital", pos: {x: 150, y: -150}, link: '../assets/sprites/hospital.png' },
        { name: "Police", pos: {x: -150, y: 150}, link: '../assets/sprites/policestation.png' },
        { name: "Fire Dept", pos: {x: 150, y: 150}, link: '../assets/sprites/firestation.png' }
    ];
    buildings.forEach((b, i) => {
        world.addObject(new Building(10 + i, b.name, 80, 80, "Service", b.pos, b.link));
    });

    // 3. Trang trí (Sử dụng nhiều loại cây khác nhau)
    const treeTypes = ['../assets/sprites/tree-1.png', '../assets/sprites/tree-2.png', '../assets/sprites/tree-3.png', '../assets/sprites/tree-4.png'];
    // Rải cây xung quanh các tòa nhà
    for(let i = 0; i < 8; i++) {
        const x = (i % 2 === 0 ? 250 : -250) + (Math.random() * 50);
        const y = (i < 4 ? 250 : -250) + (Math.random() * 50);
        world.addObject(new Decoration(100 + i, {x, y}, "Cây xanh", treeTypes[i % 4]));
    }

    // 4. Xe cộ (Sử dụng các màu xe bạn có)
    world.addObject(new Vehicle(200, {x: -200, y: 10}, 0, '../assets/sprites/car-blue.png'));
    world.addObject(new Vehicle(201, {x: 200, y: -10}, 180, '../assets/sprites/car-red.png'));
    world.addObject(new Vehicle(202, {x: -10, y: -200}, 90, '../assets/sprites/car-yellow.png'));

    // 5. Đèn giao thông (Sử dụng link gốc và code sẽ tự chuyển đổi nếu bạn nâng cấp hàm)
    world.addObject(new TrafficLight(300, '../assets/sprites/trafficlight.png', {x: 35, y: -35}, 2000, 0, "yellow"));
}

// Vòng lặp chính
function animate() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    camera.begin(ctx);
    world.draw(ctx);
    camera.end(ctx);

    requestAnimationFrame(animate);
}

init();
animate();
