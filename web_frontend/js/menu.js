/**
 * Hàm mở Sidebar và nạp file HTML tương ứng
 * @param {string} toolName - Tên công cụ (ví dụ: 'Add Building')
 */
export function openSidebar(toolName) {
    const sidebar = document.getElementById('right-sidebar');
    const contentContainer = document.getElementById('sidebar-content');

    const fileName = toolName.replace(/\s+/g, '_');

    // 1. Kích hoạt hiệu ứng trượt (thêm class active)
    sidebar.classList.add('active');

    // 2. Nạp nội dung từ file HTML tương ứng
    // Lưu ý: Phải chạy trên server (Localhost) để fetch hoạt động
    fetch('menu/' + fileName)
        .then(response => {
            if (!response.ok) {
                throw new Error('Không tìm thấy file ' + fileName);
            }
            return response.text();
        })
        .then(html => {
            contentContainer.innerHTML = html;
        })
        .catch(err => {
            contentContainer.innerHTML = `<h3>${toolName}</h3><p>Lỗi: ${err.message}</p>`;
            console.error(err);
        });
}

/**
 * Hàm đóng Sidebar
 */
export function closeSidebar() {
    const sidebar = document.getElementById('right-sidebar');
    sidebar.classList.remove('active');
}



export function prepareForm(imgSrc, name) {
    const imgEl = document.getElementById('form-img-preview');
    const inputEl = document.getElementById('form-name-input');
    
    if (imgEl) imgEl.src = imgSrc;
    if (inputEl) inputEl.value = name;
}

export function handleAdd() {
    const name = document.getElementById('form-name-input').value;
    console.log("Đã thêm:", name);
    document.querySelector('.back-action').click(); // Tự kích hoạt nút quay lại
}