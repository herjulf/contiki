from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
from SocketServer import ThreadingMixIn
import threading
import argparse
import re
import cgi
import urlparse
import json

import subprocess
import serial

# usage:
# 1) send configuration  POST /api/nodes/config -d @config.json
# 2) send command        POST /api/nodes/x -d "cmd"

# example configuration (important: moteSerialID and serialBaudrate)
# {"motes": [{
#      "httpPort" : 12100,
#      "moteNumber" : 10,
#      "moteSerialID" : "49617",
#      "serialPath" : ""                 # inserted by code in initializeNodes()
#      "serialBaudrate" : 38400,
#      "logLevel" : 4,
#      "logFile" : "log.log"
# }]
# }



def getSerialConfiguration():
  # return the connected serialIDs
  serialConfiguration = []
  for i in {2, 3, 4, 5}:
    # loop through USB interfaces
    serial = subprocess.check_output(['cat', '/sys/bus/usb/devices/1-1.%d/serial' % i]).strip()
    ps = subprocess.Popen(('ls', '/sys/bus/usb/devices/1-1.%d:1.0/' % i), stdout=subprocess.PIPE)
    path = subprocess.check_output(('grep', 'ttyUSB'), stdin=ps.stdout).strip()
    serialConfiguration.append({"serialID": serial, "serialPath": "/dev/%s" % path})
  return serialConfiguration


def initializeNodes():
  serial = getSerialConfiguration()
  for s in serial:
    for n in LocalData.nodes:
      if (n['moteSerialID'] == s['serialID']):
        n['serialPath'] = s['serialPath']
        print("found %s > %s" % (s['serialID'], s['serialPath']))

  return 200


def serialWrite(port, baudrate, data):
  ser = serial.Serial(port, baudrate, timeout=1)
  ser.write(str(data))
  ser.write('\n')
  ser.close()

 
class LocalData(object):
  records = {}
  nodes = {}
  logs = {}
 
class HTTPRequestHandler(BaseHTTPRequestHandler):
  def do_POST(self):
    if None != re.search('/api/nodes/config', self.path):
      # node configuration {nodeId, baudrate, [position, ...]}, in particular mapping nodeId and baudrate
      ctype, pdict = cgi.parse_header(self.headers.getheader('content-type'))
      length = int(self.headers.getheader('content-length'))
      data = json.loads(self.rfile.read(length))
      # TODO: sanity check ...
      LocalData.nodes = data['motes']
      print "configuration received"
      # map nodeId to usb port
      ret = initializeNodes()
      self.send_response(ret)
      self.end_headers()

    elif None != re.search('/api/nodes/*', self.path):
      # command to be sent to node n
      ctype, pdict = cgi.parse_header(self.headers.getheader('content-type'))
      length = int(self.headers.getheader('content-length'))
      data = self.rfile.read(length)
      recordID = int(self.path.split('/')[-1])
      if (recordID < len(LocalData.nodes)):
        serialWrite(LocalData.nodes[recordID]['serialPath'], LocalData.nodes[recordID]['serialBaudrate'], data)
        LocalData.records[recordID] = data
        print "sent \"%s\" to node %s" % (str(data), LocalData.nodes[recordID]['moteSerialID'])
        self.send_response(200)
        self.end_headers()
      else:
        print "node id not valid"
        self.send_response(403)
        self.end_headers()


    else:
      self.send_response(403)
      self.send_header('Content-Type', 'application/json')
      self.end_headers()
    return
 
  def do_GET(self):
    if None != re.search('/api/nodes/count', self.path):
      self.send_response(200)
#      self.send_header('Content-Type', 'application/json')
      self.end_headers()
      self.wfile.write(len(LocalData.records))

    elif None != re.search('/api/nodes', self.path):
      self.send_response(200)
#      self.send_header('Content-Type', 'application/json')
      self.end_headers()
      self.wfile.write(LocalData.records)

    elif None != re.search('/api/logs', self.path):
      self.send_response(200)
#      self.send_header('Content-Type', 'application/json')
      self.end_headers()
      self.wfile.write(LocalData.logs)

    elif None != re.search('/api/usb', self.path):
      self.send_response(200)
      self.send_header('Content-Type', 'application/json')
      self.end_headers()
      self.wfile.write(getSerialConfiguration())

    else:
      self.send_response(403)
      self.send_header('Content-Type', 'application/json')
      self.end_headers()
    return
 
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
  allow_reuse_address = True
 
  def shutdown(self):
    self.socket.close()
    HTTPServer.shutdown(self)
 
class SimpleHttpServer():
  def __init__(self, ip, port):
    self.server = ThreadedHTTPServer((ip,port), HTTPRequestHandler)
 
  def start(self):
    self.server_thread = threading.Thread(target=self.server.serve_forever)
    self.server_thread.daemon = True
    self.server_thread.start()
 
  def waitForThread(self):
    self.server_thread.join()
 
  def addRecord(self, recordID, jsonEncodedRecord):
    LocalData.records[recordID] = jsonEncodedRecord
 
  def stop(self):
    self.server.shutdown()
    self.waitForThread()
 
if __name__=='__main__':
  parser = argparse.ArgumentParser(description='HTTP Server')
  parser.add_argument('port', type=int, help='Listening port for HTTP Server')
  parser.add_argument('ip', help='HTTP Server IP')
  args = parser.parse_args()
 
  server = SimpleHttpServer(args.ip, args.port)
  print 'HTTP Server Running...........'
  server.start()
  server.waitForThread()