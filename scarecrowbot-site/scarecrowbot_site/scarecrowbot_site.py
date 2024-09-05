###### MAIN APPLICATION FILE ########
# Created for DAC by Autumn Denny.  #
# Functions of file:                #
#   - initialize pages, app flow    #
#   - initialize app settings etc.  #
#####################################

import reflex as rx
from rxconfig import config
from fastapi import FastAPI


# Defines our web app.
app = rx.App(theme = rx.theme(appearance = "light", accent_color = "tomato"),api=FastAPI(docs_url=None, redoc_url=None, openapi_url=None))

# We have to import the pages and give them URL routes.
# (see individual pages in /pages to edit them :3)
from .pages.home import home
from .pages.watch import watch

app.add_page(home, route = '/')
app.add_page(watch, route = '/watchnow')

# For API calls
from .api_calls.api import stopGame, startGame, getStatus, pushScore
app.api.add_api_route("/api/stopGame", stopGame)
app.api.add_api_route("/api/startGame", startGame)
app.api.add_api_route("/api/getGameStatus", getStatus)
app.api.add_api_route("/api/pushScore", pushScore)