from __future__ import annotations
import argparse
import atexit
from flask import Flask, Response, jsonify, request, render_template
from camera import CameraStreamer
from controller import RobotController

def create_app(controller: RobotController, camera: CameraStreamer) -> Flask:
    app = Flask(__name__)
    @app.get("/")
    def index(): return render_template("index.html")
    @app.get("/video_feed")
    def video_feed():
        if not camera.online: return jsonify(error=camera.error or camera.status), 503
        return Response(camera.iter_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")
    @app.post("/api/keys")
    def keys(): controller.update_keys(request.get_json(silent=True) or {}); return jsonify(ok=True)
    @app.post("/api/stop")
    def stop(): controller.stop_now(); return jsonify(ok=True)
    @app.post("/api/config")
    def config(): return jsonify(ok=True, config=controller.update_config(request.get_json(silent=True) or {}))
    @app.get("/api/status")
    def status(): return jsonify(robot=controller.status(), camera={"online":camera.online,"status":camera.status,"error":camera.error})
    return app

def main() -> None:
    parser=argparse.ArgumentParser(); parser.add_argument("--port",default="/dev/ttyACM0"); parser.add_argument("--web-port",type=int,default=5000); args=parser.parse_args()
    camera=CameraStreamer(); camera.start(); controller=RobotController(args.port); controller.start(); atexit.register(camera.stop)
    create_app(controller,camera).run(host="0.0.0.0",port=args.web_port,threaded=True,use_reloader=False)
if __name__ == "__main__": main()
