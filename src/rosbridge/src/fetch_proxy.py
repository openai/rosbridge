#!/usr/local/bin/python
import math, random, time, logging, re, base64, argparse, collections, sys, os
import numpy as np
import ujson
from wand.image import Image
import gym
import wsaccel
wsaccel.patch_ws4py()
import cherrypy
from ws4py.server.cherrypyserver import WebSocketPlugin, WebSocketTool
from ws4py.websocket import WebSocket

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class GymProxyServer(WebSocket):
    def opened(self):
        logger.info('GymProxyServer opened')
        self.start_robot()
        # WRITEME: fire up ROS, etc.
        pass

    def received_message(self, message):
        rpc = ujson.loads(message.data)
        logger.info('rpc > %s', rpc)
        rpc_method = rpc.get('method', None)
        rpc_params = rpc.get('params', None)
        rpc_id = rpc.get('id', None)
        def reply(result, error=None):
            rpc_out = ujson.dumps({
                'id': rpc_id,
                'error': error,
                'result': result,
            })
            self.send(rpc_out)
        try:
            if rpc_method == 'reset':
                reply(self.handle_reset(rpc_params))
            elif rpc_method == 'step':
                reply(self.handle_step(rpc_params))
            elif rpc_method == 'close':
                self.close(reason='requested')
            else:
                raise Exception('unknown method %s' % rpc_method)
        except:
            ex = sys.exc_info()[0]
            logger.error('rpc_method=%s ex=%s', rpc_method, ex)
            reply(None, ex)

    def closed(self, code, reason=None):
        logger.info('GymProxyServer closed %s %s', code, reason)
        pass

    def start_robot(self):
        # override me
        pass

    def handle_reset(self, params):
        # override me
        return {
            'obs': [0.0],
        }

    def handle_step(self, params):
        # override me
        return {
            'obs': [0.0],
            'reward': 0.0,
            'info': {},
            'done': False,
        }



cherrypy.config.update({'server.socket_port': 9000})
WebSocketPlugin(cherrypy.engine).subscribe()
cherrypy.tools.websocket = WebSocketTool()

class WebRoot(object):
    @cherrypy.expose
    def index(self):
        return 'some HTML with a websocket javascript connection'

    @cherrypy.expose
    def fr(self):
        # you can access the class instance through
        handler = cherrypy.request.ws_handler

cherrypy.quickstart(WebRoot(), '/', config={'/fr': {
    'tools.websocket.on': True,
    'tools.websocket.handler_cls': GymProxyServer
}})
