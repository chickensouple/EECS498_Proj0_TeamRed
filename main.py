from joy import *
import time

numMotors = 1;


class MainApp(JoyApp):
	""" Main app to run the program that controls our robot """
	def __init__(self, *arg, **kw):
		cfg = dict()
		JoyApp.__init__(self, cfg=cfg, *arg, **kw)


	def onStart(self):
		pass

	def onEvent(self, evt):
		print(evt)




robot = {"count": numMotors}
scr = {}

app = MainApp(robot=robot, scr=scr)
app.run()
