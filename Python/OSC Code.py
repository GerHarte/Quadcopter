import socket
import OSC
import time, threading
import serial


receive_address = '192.168.1.11', 8000
 
s = OSC.OSCServer(receive_address) # basic
s.addDefaultHandlers()

ser = serial.Serial('/dev/tty.usbmodem641', 115200, timeout = 1)

p_Val = 0
i_Val = 0
d_Val = 0
throttle_Val = 0
last_JSON = ''
start_time = time.time()
count = 0

# define a message-handler function for the server to call.
def printing_handler(addr, tags, stuff, source):
  global p_Val 
  global i_Val
  global d_Val
  global throttle_Val
  global start_time
  global last_JSON
  global count
  if count == 0:
    count = count + 1
  if addr == "/1/pfader":
    p_Val = round(stuff[0], 2)
  elif addr == "/1/ifader":
    i_Val = round(stuff[0], 2)
  elif addr == "/1/dfader":
    d_Val = round(stuff[0], 2)
  elif addr == "/1/throttlefader":
    throttle_Val = round(stuff[0], 2)
  json = "{\"throttle\":" + str(throttle_Val) + ",  \"p\":" + str(p_Val) + ",  \"i\":" + str(i_Val) + ",  \"d\":" + str(d_Val) + "}"
  if (count != 1 and start_time - time.time() > 2):
    ser.write(json)
    print(json)
    count = 0
  else:
    "Nothing Transmitted"

s.addMsgHandler("/1/pfader", printing_handler) # adding our function
s.addMsgHandler("/1/ifader", printing_handler) # adding our function
s.addMsgHandler("/1/dfader", printing_handler) # adding our function
s.addMsgHandler("/1/throttlefader", printing_handler) # adding our function
 
# just checking which handlers we have added
print "Registered Callback-functions are :"
for addr in s.getOSCAddressSpace():
    print addr


# Start OSCServer
print "\nStarting OSCServer. Use ctrl-C to quit."
st = threading.Thread( target = s.serve_forever )
st.start()


try:
  while 1:
    time.sleep(5)
except KeyboardInterrupt:
  print "\nClosing OSCServer."
  s.close()
  print "Waiting for Server-thread to finish"
  st.join() ##!!!
  print "Done"