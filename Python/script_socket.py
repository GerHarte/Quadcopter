#!/usr/bin/env python
import cgi
import json
import socket

# Read the speed from index.html
form=cgi.FieldStorage()

# Open a socket connection
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Connect the socket over localhost on port 9988
s.connect(("localhost", 9988))
# Send the value from index.html as a string over the socket
s.sendall(str(form["value"].value))
# Close the connection
s.close()

# Close out the CGI script by sending a successful status
print "Content-type: application/json"
print
print(json.JSONEncoder().encode({"status":"ok"}))
