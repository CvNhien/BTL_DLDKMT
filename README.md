# BTL_DLDKMT
BTL_DLDKMT
STM32F4 giao tiếp với máy tính thông qua thiết bị CP2120 (USB to UART)

![image](https://github.com/CvNhien/BTL_DLDKMT/assets/111190445/c607673d-9ef5-4e84-a5a6-81a008734fe5)

Firmware:
  Sử dụng KeliC lập trình STM32F4, thư viện std
  + UART
  + ADC (đọc tín hiệu từ mạch xử lý cảm biến nhiệt độ PT100)
  + DAC (Tạo xung điều khiển động cơ SG90-180độ)
  + DMA + interupt
  + Digital Output
  + Digital Input
  + External Interupt

Software:
  Qt creator 5.12 (C++)
  
 ![image](https://github.com/CvNhien/BTL_DLDKMT/assets/111190445/f2270ba4-1b03-426e-8704-3e33340515f8)
 
- Group UART
Chọn các thông số cấu hình UART phù hợp với thiết lập của Vi điều khiển STM32
•	Connect: Để kết nối
•	Disconnect: Ngừng kết nối
•	Refresh Ports: Xóa cấu hình UART
- Group Static Command
•	ADC: Command = “ADC”
•	DAC: Command = “DAC”
•	DO: Command = “GDO”
•	DI: Command = “GDI”
- Group Control
•	Timer: Gửi chuỗi mỗi 1 giây
•	Event: Gửi theo sự kiện
- Group ADC
•	Hiển thị giá trị nhiệt độ
- Group Digital Input
•	Đảo trạng thái mỗi khi nhấn nút bấm PA0
- Group Digital Output
•	Trạng thái có Led trên Kit tương ứng.
- TextBox: Hiển thị dữ liệu truyền và nhận
•	Dữ liệu gửi xuống Kit: Màu xanh dương
•	Dữ liệu nhận từ Kit: Mùa xanh lá
- Nút Clear: Xóa dữ liệu trên TextBox
- Nút Stop: Dừng hoạt động, trở về trạng thái mặc định.
- LineEdit “Data”: Nhập dữ liệu mong muốn

 
Handshake protocol:

![image](https://github.com/CvNhien/BTL_DLDKMT/assets/111190445/930fc53a-d7f4-4c16-a850-63dd384429e1)


