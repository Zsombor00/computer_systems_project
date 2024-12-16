import os
from http.server import SimpleHTTPRequestHandler, HTTPServer


class MyHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/run-regular-mode":
            try:
                # Run the Python script (adjust filename as needed)
                #os.system("python3 myscript.py")
                os.system("bash /home/ubuntu/group2/myagv_ros/src/htmlFolder/run_ros.sh")
                # Send a response back to the browser
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b"Regular mode script executed successfully!")
            except Exception as e:
                # Send an error response if script fails
                self.send_response(500)
                self.end_headers()
                self.wfile.write(f"Error running script: {e}".encode())
        else:
            # Serve other files like the HTML
            super().do_GET()

# Start the server
PORT = 8001
server_address = ("", PORT)
httpd = HTTPServer(server_address, MyHandler)
print(f"Server running on http://localhost:{PORT}")
httpd.serve_forever()