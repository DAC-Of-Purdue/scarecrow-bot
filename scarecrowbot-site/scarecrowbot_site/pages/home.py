###### HOME PAGE ####################
# Created for DAC by Autumn Denny.  #
# Functions of page:                #
#   - present leaderboard (scores)  #
#   - offer login or register opts  #
#####################################

import reflex as rx
from scarecrowbot_site.global_components import footer
from scarecrowbot_site.global_components import wrapper
import mysql.connector
from typing import List, Tuple

#### Get information from the MySQL database to put into the table. ###
class LeaderboardState(rx.State):
    _data: List[Tuple[int, str, str, str, str]] = []

    @rx.var
    def data(self) -> List[Tuple[int, str, str, str, str]]:
        return self._data

    def fetchTeamData(self):
        with mysql.connector.connect(host="localhost", user="rabbitadmin", password="24rRatzRo0l", database="scarecrowbot") as database:
            with database.cursor() as cursor:
                cursor.execute("SELECT * FROM TEAMS ORDER BY HighestScore DESC")
                self._data = cursor.fetchall()

#### Get information regarding game status to allow/disallow new logins/games. ###
class GameState(rx.State):
    _data: List[Tuple[int,int]] = []
    _status: Tuple[int,int] = (0, "")

    @rx.var
    def data(self) -> List[Tuple[int,int]]:
        return self._data

    @rx.var
    def status(self) -> Tuple[int,int]:
        return self._status

    # Changed True to False here
    @rx.var
    def allowLogin(self) -> bool:
        return self._data[0][0] != 1 if self._data else False

    @rx.var
    def teamName(self) -> str:
        if self._data and self.data[0][0]==1:
            with mysql.connector.connect(host="localhost", user="rabbitadmin", password="24rRatzRo0l", database="scarecrowbot") as database:
                with database.cursor() as cursor:
                    query = "SELECT Name FROM TEAMS WHERE TeamID = " + str(self._data[0][1])
                    cursor.execute(query)
                    team = cursor.fetchall()
                    teamName = team[0][0]
            return(teamName)
        else:
            return ""

    def getStatus(self):
        with mysql.connector.connect(host="localhost", user="rabbitadmin", password="24rRatzRo0l", database="scarecrowbot") as database:
            with database.cursor() as cursor:
                cursor.execute("SELECT * FROM GAME_STATUS")
                self._data = cursor.fetchall()
                self._status = self._data[0] if self._data else (0, "")


class LoginState(rx.State):
    _data: List[Tuple[int,str,str, str,str]] = []
    formData = {}

    def fetchTeamInfo(self, formData: dict):
        self.formData = formData
        loginPIN = formData["login_pin"]
        with mysql.connector.connect(host="localhost", user="rabbitadmin", password="24rRatzRo0l", database="scarecrowbot") as database:
            with database.cursor() as cursor:
                cursor.execute("SELECT * FROM TEAMS WHERE LoginPIN = " + str(loginPIN))
                self._data = cursor.fetchall()
        if self._data:
            return LoginState.startGame

    def startGame(self):
        with mysql.connector.connect(host="localhost", user="rabbitadmin", password="24rRatzRo0l", database="scarecrowbot") as database:
            with database.cursor() as cursor:
                query = "UPDATE GAME_STATUS SET Status = True, CurrentTeam=" + str(self._data[0][0]) +  " where Status = False"
                cursor.execute(query)
            database.commit()
        return GameState.getStatus()
        #GameState.allowLogin = Fals

#### Defining the home components (grouped UI elements) here, for simplicity. ###

# Login PIN form.
def noGameStatus() -> rx.Component: 
    return rx.card(
        rx.vstack(
            rx.heading("Sign your team in using your login PIN."),
            rx.form(
                rx.hstack(
                    rx.input(placeholder="Login PIN", name="login_pin", type="password"),
                    rx.button("Submit", type="submit", size="3"),
                    width="100%",
                    spacing="4",
                    align_content = "center",
                    justify_content = "center",
                    align_items = "center",
                    justify_items = "center",
                    align = "center",
                    justify = "center",
                ),
                align_content = "center",
                justify_content = "center",
                align_items = "center",
                justify_items = "center",
                align = "center",
                justify = "center",
                width="100%",
                on_submit = LoginState.fetchTeamInfo
            ),
            rx.text("When you log in your team, the game will begin!"),
            width="100%",
            spacing="4",
            align_content = "center",
            justify_content = "center",
            align_items = "center",
            justify_items = "center",
            align = "center",
            justify = "center"
        ),
        height="100%",
        width="100%",
        style={
            "backgroundColor": "rgba(255, 255, 255, 1.0)",
        },
        variant="ghost",
        align_content = "center",
        justify_content = "center",
        align_items = "center",
        justify_items = "center",
        align = "center",
        justify = "center"
    )

def inProgressStatus() -> rx.Component:
    return rx.card(
        rx.vstack(
            rx.heading("There's a game in progress"),
            rx.text(GameState.teamName + " is currently playing! Refresh this page in a bit if you are waiting to start a game."),
            rx.link(rx.button("Watch Now"), href="https://rabbitrun.digitalagclub.org/watchnow"),
            width="100%",
            spacing="4",
            align_content = "center",
            justify_content = "center",
            align_items = "center",
            justify_items = "center",
            align = "center",
            justify = "center"
        ),
        height="100%",
        width="100%",
        style={
            "backgroundColor": "rgba(255, 255, 255, 1.0)",
        },
        variant="ghost",
        align_content = "center",
        justify_content = "center",
        align_items = "center",
        justify_items = "center",
        align = "center",
        justify = "center"
    )

def gameStatus() -> rx.Component:
    return rx.cond(
            GameState.allowLogin,
            noGameStatus(),
            inProgressStatus(),
        ),

# Leaderboard
def leaderboard() -> rx.Component:
    return rx.card(
        rx.table.root(
            rx.table.header(
                rx.table.row(
                    rx.table.column_header_cell("Team Name"),
                    rx.table.column_header_cell("Highest Score")
                ),
            ),
            rx.table.body(
                rx.foreach(
                    LeaderboardState.data,
                    lambda row: rx.table.row(
                        rx.table.row_header_cell(row[1]),
                        rx.table.cell(row[3])
                    )
                )
            ),
        ),
        style={
            "backgroundColor": "rgba(255, 255, 255, 1.0)",
        },
        variant="ghost",
        width="100%",
        size="2",
    )

# Current team and player.
def home():
    return(
        wrapper.globalWrapper(
            gameStatus(),
            leaderboard(),
        )
    )