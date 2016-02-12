#!/usr/bin/python3
# -*- coding: utf-8 -*-

import time
import threading
import serial
import cairo
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GLib

class LivePlot(Gtk.Window):

    def __init__(self):
        Gtk.Window.__init__(self, title="LivePlot")

        self.raw = serial.Serial("/dev/ttyACM0",115200, rtscts=1)
        self.raw.close()
        self.raw.rts = 0
        self.raw.dtr = 0
        self.raw.open()

        self.connect('destroy', lambda w: Gtk.main_quit())
        self.set_default_size(1920, 1080)

        self.drawingarea = Gtk.DrawingArea()
        self.add(self.drawingarea)
        self.drawingarea.connect('draw', self.draw)

        self.show_all()

        GLib.timeout_add(1, self.queue)

    def queue(self):
        self.drawingarea.queue_draw_area(0, 0, 1920, 1080)
        return True

    def draw(self, widget, context):

        try:
            x, y, z = [float(var) for var in str(self.raw.readline()).split(', ')[1:4]]
            w, h = self.get_size()

            sc = (h/2)*0.9

            context.translate(w/2, h/2)
            context.set_line_cap(cairo.LINE_CAP_ROUND)

            # coordinate system
            context.set_line_width(5)
            context.set_source_rgb(0.129, 0.129, 0.129)

            # x
            context.move_to(0, 0)
            context.line_to(-0.87*sc, 0.50*sc)
            context.stroke()
            # y
            context.move_to(0, 0)
            context.line_to(0, 1*sc)
            context.stroke()
            # z
            context.move_to(0, 0)
            context.line_to(0.87*sc, 0.50*sc)
            context.stroke()

            # bars
            context.set_line_width(5)
            context.set_source_rgb(1, 0.341, 0.133)
            # x
            context.move_to(0, 0)
            context.line_to(0.87*y*sc, 0.50*-y*sc)
            context.stroke()
            # y
            context.move_to(0, 0)
            context.line_to(0, 1*z*sc)
            context.stroke()
            # z
            context.move_to(0, 0)
            context.line_to(0.87*x*sc, 0.50*x*sc)
            context.stroke()

        except:
            pass

if __name__ == '__main__':
    plot = LivePlot()
    Gtk.main()
