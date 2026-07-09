import { openSidebar, closeSidebar } from './menu.js';

window.openSidebar = openSidebar;
window.closeSidebar = closeSidebar;

import { prepareForm, handleAdd } from './menu.js';

window.prepareForm = prepareForm;
window.handleAdd = handleAdd;

//import { add } from './features.js';
//
//window.add = add;

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
    world.addObject(new Road(1, "Main Street", {x: -300, y: 0}, {x: 300, y: 0}, 60));
    world.addObject(new Intersection(2, "Giao lộ A", {x: 0, y: 0}, 4, 0, 60));
    world.addObject(new Building(3, "Tòa nhà X", 80, 80, "Office", {x: 100, y: -100}, '../assets/building.png'));
    world.addObject(new Vehicle(4, {x: -200, y: 0}, 0, '../assets/car.png'));
    world.addObject(new TrafficLight(5, '../assets/light.png', {x: 50, y: -50}, 1000, 0, "green"));
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