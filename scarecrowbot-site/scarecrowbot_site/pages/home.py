###### HOME PAGE ####################
# Created for DAC by Autumn Denny.  #
# Functions of page:                #
#   - present leaderboard (scores)  #
#   - offer login or register opts  #
#####################################

import reflex as rx


#### Defining the home components (grouped UI elements) here, for simplicity. ###

# Leaderboard (shows teams and scores)

# Login or register option buttons.
def loginOptions() -> rx.Component:
    return rx.card(
        rx.vstack(
            rx.heading("Get your team in the game!"),
            rx.hstack(
                rx.button("Team Sign-in", size = '3'),
                rx.box(rx.text(" or ")),
                rx.button("Register Team", size = '3'),
                justify_content = "center",
                align_items = "center",
                justify = "center",
                width = "100%"
            ),
            align_items = "center",
            justify_content = "center",
            height = "100%"
        ),
        style={
            "backgroundColor": "rgba(0, 0, 0, 0.8)",
            "color": "white",
        },
        variant = "ghost"
    )

# Login PIN form.
    return rx.card(
        rx.vstack(
            rx.heading("Sign your team in using your login PIN."),
            rx.form(
                rx.hstack(
                    rx.input(name = "Login PIN", type = "password"),
                    rx.button("Submit", size = "3")
                )
            ),
            align_items = "center",
            justify_content = "center",
            height = "100%"
        ),
        style={
            "backgroundColor": "rgba(0, 0, 0, 0.8)",
            "color": "white",
        },
        variant = "ghost"
    )

# Current team and player.

### Secondary container (keeps the leaderboard and login options constrained to a specified width).
## Takes arguments in the form of an array of functions (other components to include in this parent container).
def secondaryContainer(*components) -> rx.Component:
    return rx.container(
        *components,
        size = '4',
        justify_content = 'center',
    )

### Main container wrapper (has background image).
def bkgrndContainer(*components) -> rx.Component:
    return rx.container(
        *components,
        width = "100%",
        height = "100vh",
        background = "url('/bkgrnd.png')",
        background_size = "cover",
        background_position = "center",
        justify_content = "center"
    )


def home():
    return bkgrndContainer(
        secondaryContainer(
            loginOptions()
        )
    )
