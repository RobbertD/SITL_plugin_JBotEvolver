import socket

HEADERSIZE = 10

# def find_free_port():
#     s = socket.socket()
#     s.bind(('', 0))            # Bind to a free port provided by the host.
#     return s.getsockname()[1]

# print(find_free_port())

r = 4
print(len(bytes(str(r), 'utf-8')))

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('localhost', 59800))
s.send(bytes('2','utf-8'))

for i in range(30):
    msg = ''
    while msg == '':
        msg = s.recv(67).decode("utf-8")

    sensor_readings = msg

    print('received sensor readings: {}'.format(sensor_readings))

    angle = '0'
    s.send(bytes(angle, 'utf-8'))
    print('sent angle: {}'.format(angle))

s.send(bytes('3','utf-8'))

s.shutdown(2)