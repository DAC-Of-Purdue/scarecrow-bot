###### MAIN APPLICATION FILE ########
# Created for DAC by Autumn Denny.  #
# Functions of file:                #
#   - initialize pages, app flow    #
#   - initialize app settings etc.  #
#####################################

import reflex as rx
from rxconfig import config

# Defines our web app.
app = rx.App(theme = rx.theme(appearance = "light", panel_background = "translucent"))

# We have to import the pages and give them URL routes.
# (see individual pages in /pages to edit them :3)
from .pages.home import home
from .pages.registerteam import register

app.add_page(home, route = '/')
app.add_page(register, route = '/register')
