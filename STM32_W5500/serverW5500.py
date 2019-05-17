#!/usr/bin/env python2
import time
import socket
def parse(conn, addr):#processing a connection in a separate function
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                else:
                    print(data)
                    conn.send("OKsssssssssssss")
                  

sock = socket.socket()                              
sock.bind( ("", 9095) )
sock.listen(5)

try:
    while 1: # we work constantly
        conn, addr = sock.accept()
        print("New connection from ",addr)
        try:
            parse(conn, addr)
        except socket.error, e:
            print('ERROR! Exception {}'.format(e))
        finally:
            conn.close()
finally: sock.close()

