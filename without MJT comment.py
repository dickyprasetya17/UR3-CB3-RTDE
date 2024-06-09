"""
Code written based on servoj example from: https://github.com/davizinho5/RTDE_control_example
"""  # Komentar yang menjelaskan bahwa kode ini ditulis berdasarkan contoh servoj dari tautan yang diberikan

import sys  # Mengimpor modul sistem
sys.path.append('')  # Menambahkan direktori saat ini ke dalam jalur sistem
import logging  # Mengimpor modul logging
import rtde as rtde  # Mengimpor modul rtde dari paket ur_rtde yang telah terinstal
from ur_rtde import rtde_config  # Mengimpor modul rtde_config dari paket ur_rtde yang telah terinstal
import time  # Mengimpor modul time
from matplotlib import pyplot as plt  # Mengimpor modul pyplot dari paket matplotlib untuk pembuatan grafik

# -------- functions -------------
def setp_to_list(setp):  # Mendefinisikan fungsi untuk mengonversi objek setp menjadi daftar
    temp = []  # Membuat daftar sementara kosong
    for i in range(0, 6):  # Perulangan dari 0 hingga 5
        temp.append(setp.__dict__["input_double_register_%i" % i])  # Menambahkan nilai register double ke daftar sementara
    return temp  # Mengembalikan daftar sementara

def list_to_setp(setp, list):  # Mendefinisikan fungsi untuk mengonversi daftar menjadi objek setp
    for i in range(0, 6):  # Perulangan dari 0 hingga 5
        setp.__dict__["input_double_register_%i" % i] = list[i]  # Menetapkan nilai dari daftar ke register double pada objek setp
    return setp  # Mengembalikan objek setp yang telah diperbarui

# ------------- robot communication stuff -----------------
ROBOT_HOST = '10.29.202.104'  # Alamat IP robot
ROBOT_PORT = 30004  # Port robot
config_filename = r'C:\Users\dicky\Downloads\Compressed\RTDE_Python_Client_Library-main\Servoj_RTDE_UR3-main\control_loop_configuration.xml'  # Lokasi file konfigurasi XML untuk sinkronisasi data

logging.getLogger().setLevel(logging.INFO)  # Mengatur tingkat logging ke INFO

conf = rtde_config.ConfigFile(config_filename)  # Membuat objek ConfigFile dari file konfigurasi
state_names, state_types = conf.get_recipe('state')  # Mendapatkan daftar nama dan tipe untuk resep 'state' (output robot)
setp_names, setp_types = conf.get_recipe('setp')  # Mendapatkan daftar nama dan tipe untuk resep 'setp' (input robot)
watchdog_names, watchdog_types = conf.get_recipe('watchdog')  # Mendapatkan daftar nama dan tipe untuk resep 'watchdog'

# -------------------- Establish connection--------------------
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)  # Membuat objek RTDE dengan alamat IP dan port robot
connection_state = con.connect()  # Mencoba menghubungkan ke robot

# check if connection has been established
while connection_state != 0:  # Perulangan hingga koneksi berhasil
    time.sleep(0.5)  # Menunggu selama 0,5 detik
    connection_state = con.connect()  # Mencoba menghubungkan ke robot
print("---------------Successfully connected to the robot-------------\n")  # Mencetak pesan jika koneksi berhasil

# get controller version
con.get_controller_version()  # Mendapatkan versi kontroler robot

# ------------------- setup recipes ----------------------------
FREQUENCY = 500  # Frekuensi pengiriman data sebesar 500 Hz
con.send_output_setup(state_names, state_types, FREQUENCY)  # Menyiapkan resep output dengan frekuensi 500 Hz
setp = con.send_input_setup(setp_names, setp_types)  # Menyiapkan resep input setp
watchdog = con.send_input_setup(watchdog_names, watchdog_types)  # Menyiapkan resep input watchdog

setp.input_double_register_0 = 0  # Menetapkan nilai awal register double 0 pada objek setp
setp.input_double_register_1 = 0  # Menetapkan nilai awal register double 1 pada objek setp
setp.input_double_register_2 = 0  # Menetapkan nilai awal register double 2 pada objek setp
setp.input_double_register_3 = 0  # Menetapkan nilai awal register double 3 pada objek setp
setp.input_double_register_4 = 0  # Menetapkan nilai awal register double 4 pada objek setp
setp.input_double_register_5 = 0  # Menetapkan nilai awal register double 5 pada objek setp

setp.input_bit_registers0_to_31 = 0  # Menetapkan nilai awal register bit 0 hingga 31 pada objek setp

watchdog.input_int_register_0 = 0  # Menetapkan nilai awal register integer 0 pada objek watchdog

# start data synchronization
if not con.send_start():  # Jika gagal memulai sinkronisasi data
    sys.exit()  # Keluar dari program

start_pose =  [-0.3, -0.44, 0.1614904966558569, -0.1, -3.1, 0.1]  # Posisi awal robot
desired_pose = [-0.1, -0.35, 0.2, -0.1, -3.1, 0.1]  # Posisi tujuan yang diinginkan

state = con.receive()  # Menerima data keadaan robot
tcp1 = state.actual_TCP_pose  # Mendapatkan posisi TCP saat ini
print(tcp1)  # Mencetak posisi TCP saat ini

#   ------------  mode = 1 (Connection) -----------
while True:  # Perulangan untuk menunggu pengguna mengklik CONTINUE pada Polyscope
    print('Boolean 1 is False, please click CONTINUE on the Polyscope')  # Mencetak instruksi
    state = con.receive()  # Menerima data keadaan robot
    con.send(watchdog)  # Mengirim objek watchdog ke robot
    if state.output_bit_registers0_to_31:  # Jika register bit output 0 hingga 31 bernilai True
        print('Boolean 1 is True, Robot Program can proceed to mode 1\n')  # Mencetak pesan
        break  # Keluar dari perulangan

print("-------Executing moveL -----------\n")  # Mencetak pesan

watchdog.input_int_register_0 = 1  # Menetapkan mode 1 pada register integer 0 pada objek watchdog
con.send(watchdog)  # Mengirim objek watchdog ke robot
list_to_setp(setp, start_pose)  # Mengubah posisi awal pada objek setp
con.send(setp)  # Mengirim objek setp ke robot

while True:  # Perulangan untuk menunggu gerakan moveL selesai
    print('Waiting for moveL() to finish')  # Mencetak pesan
    state = con.receive()  # Menerima data keadaan robot
    con.send(watchdog)  # Mengirim objek watchdog ke robot
    if not state.output_bit_registers0_to_31:  # Jika register bit output 0 hingga 31 bernilai False
        print('Proceeding to mode 2\n')  # Mencetak pesan
        break  # Keluar dari perulangan

print("-------Executing moveL  -----------\n")  # Mencetak pesan

watchdog.input_int_register_0 = 2  # Menetapkan mode 2 pada register integer 0 pada objek watchdog
con.send(watchdog)  # Mengirim objek watchdog ke robot

# Inisialisasi variabel untuk menyimpan data
time_data = []  # Daftar kosong untuk menyimpan data waktu
tcp_pos_x = []  # Daftar kosong untuk menyimpan data posisi TCP sumbu X
tcp_pos_y = []  # Daftar kosong untuk menyimpan data posisi TCP sumbu Y
tcp_pos_z = []  # Daftar kosong untuk menyimpan data posisi TCP sumbu Z
tcp_vel_x = []  # Daftar kosong untuk menyimpan data kecepatan TCP sumbu X
tcp_vel_y = []  # Daftar kosong untuk menyimpan data kecepatan TCP sumbu Y
tcp_vel_z = []  # Daftar kosong untuk menyimpan data kecepatan TCP sumbu Z

# Menggerakkan robot langsung ke desired_pose dengan moveL
list_to_setp(setp, desired_pose)  # Menetapkan desired_pose pada objek setp
con.send(setp)  # Mengirim objek setp ke robot

start_time = time.time()  # Mendapatkan waktu awal
max_time = 2  # Batasan waktu maksimum dalam detik

while True:  # Perulangan untuk mengambil data sampai mencapai desired_pose atau waktu maksimum tercapai
    state = con.receive()  # Menerima data keadaan robot
    con.send(watchdog)  # Mengirim objek watchdog ke robot

    elapsed_time = time.time() - start_time  # Menghitung waktu yang telah berlalu
    time_data.append(elapsed_time)  # Menambahkan waktu yang telah berlalu ke daftar time_data

    actual_tcp_pose = state.actual_TCP_pose  # Mendapatkan posisi TCP saat ini
    tcp_pos_x.append(actual_tcp_pose[0])  # Menambahkan posisi TCP sumbu X ke daftar tcp_pos_x
    tcp_pos_y.append(actual_tcp_pose[1])  # Menambahkan posisi TCP sumbu Y ke daftar tcp_pos_y
    tcp_pos_z.append(actual_tcp_pose[2])  # Menambahkan posisi TCP sumbu Z ke daftar tcp_pos_z

    actual_tcp_speed = state.actual_TCP_speed  # Mendapatkan kecepatan TCP saat ini
    tcp_vel_x.append(actual_tcp_speed[0])  # Menambahkan kecepatan TCP sumbu X ke daftar tcp_vel_x
    tcp_vel_y.append(actual_tcp_speed[1])  # Menambahkan kecepatan TCP sumbu Y ke daftar tcp_vel_y
    tcp_vel_z.append(actual_tcp_speed[2])  # Menambahkan kecepatan TCP sumbu Z ke daftar tcp_vel_z

    if state.actual_TCP_pose == desired_pose:  # Jika posisi TCP saat ini sama dengan desired_pose
        break  # Keluar dari perulangan

    # Keluar dari loop jika waktu maksimum tercapai
    if elapsed_time > max_time:  # Jika waktu yang telah berlalu melebihi waktu maksimum
        print(f"Waktu maksimum ({max_time} detik) tercapai sebelum mencapai desired_pose.")  # Mencetak pesan
        break  # Keluar dari perulangan

print(f"Robot telah digerakkan ke desired_pose: {desired_pose}")  # Mencetak pesan

state = con.receive()  # Menerima data keadaan robot
print('--------------------\n')
print('Actual Current TCP Pose:\n')
print(state.actual_TCP_pose)  # Mencetak posisi TCP saat ini

# ====================mode 3===================
watchdog.input_int_register_0 = 3  # Menetapkan mode 3 pada register integer 0 pada objek watchdog
con.send(watchdog)  # Mengirim objek watchdog ke robot

con.send_pause()  # Mengirim perintah pause ke robot
con.disconnect()  # Memutuskan koneksi dengan robot

# Plot data posisi dan kecepatan TCP
fig1, ax1 = plt.subplots()  # Membuat subplot baru untuk plot posisi TCP sumbu X
ax1.plot(time_data, tcp_pos_x, color='orange')  # Memplot data posisi TCP sumbu X
ax1.set_title('TCP Position X')  # Menetapkan judul plot
ax1.set_xlabel('Time (s)')  # Menetapkan label sumbu X
ax1.set_ylabel('Position (m)')  # Menetapkan label sumbu Y
ax1.grid()  # Menambahkan grid pada plot

# Langkah-langkah serupa dilakukan untuk membuat subplot dan memplot data posisi dan kecepatan TCP sumbu Y dan Z

plt.tight_layout()  # Mengatur tata letak plot agar tidak terpotong
plt.show()  # Menampilkan semua plot