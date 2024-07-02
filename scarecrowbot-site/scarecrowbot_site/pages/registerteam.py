###### REGISTRATION PAGE ############
# Created for DAC by Autumn Denny.  #
# Functions of page:                #
#   - register new teams            #
#   - customize team color and icon #
#####################################

import reflex as rx


#### Defining the registration page components (grouped UI elements) here, for simplicity. ###

### Registration info container
def registrationForm(*components) -> rx.Component:
    return rx.card(
        rx.center(rx.heading("Register a new team")),
        rx.form(
            rx.text("Team Name"),
            rx.input(name = "teamName", required = True),
            rx.text("Login PIN"),
            rx.input(name = "loginPIN", type = "password", required = True, placeholder = "4-digit PIN"),
            # These are the team color buttons VVV
            rx.text("Team Color"),
            rx.chakra.button_group(
                # red
                rx.chakra.button(
                    size = "sm",
                    bg = "#ed1f2d",
                    _hover={
                        "border": "2px solid white",
                        "bg": "#ed1f2d",
                    },
                    _active={
                        "border": "2px solid white",
                        "bg": "#ed1f2d",
                    }),
                # orange
                rx.chakra.button(
                    size = "sm",
                    bg = "#ed9b1f",
                    _hover={
                        "border": "2px solid white",
                        "bg": "#ed9b1f",
                    },
                    _active={
                        "border": "2px solid white",
                        "bg": "#ed9b1f",
                    }),
                # yellow
                rx.chakra.button(
                    size = "sm",
                    bg = "#fcf80d",
                    _hover={
                        "border": "2px solid white",
                        "bg": "#fcf80d",
                    },
                    _active={
                        "border": "2px solid white",
                        "bg": "#fcf80d",
                    }),
                # green
                rx.chakra.button(
                    size = "sm",
                    bg = "#39e609",
                    _hover={
                        "border": "2px solid white",
                        "bg": "#39e609",
                    },
                    _active={
                        "border": "2px solid white",
                        "bg": "#39e609",
                    }),
                # blue
                rx.chakra.button(
                    size = "sm",
                    bg = "#1873f2",
                    _hover={
                        "border": "2px solid white",
                        "bg": "#1873f2",
                    },
                    _active={
                        "border": "2px solid white",
                        "bg": "#1873f2",
                    }),
                # purple
                rx.chakra.button(
                    size = "sm",
                    bg = "#d211f0",
                    _hover={
                        "border": "2px solid white",
                        "bg": "#d211f0",
                    },
                    _active={
                        "border": "2px solid white",
                        "bg": "#d211f0",
                    }),
                # white
                rx.chakra.button(
                    size = "sm",
                    bg = "#ffffff",
                    _hover={
                        "border": "2px solid white",
                        "bg": "#ededed",
                    },
                    _active={
                        "border": "2px solid white",
                        "bg": "#ededed",
                    }),
                # pink
                rx.chakra.button(
                    size = "sm",
                    bg = "#ff7373",
                    _hover={
                        "border": "2px solid white",
                        "bg": "#ff7373",
                    },
                    _active={
                        "border": "2px solid white",
                        "bg": "#ff7373",
                    }),
            ),
            # These are the team icon options.
            rx.text("Team Icon"),
            rx.vstack(
                rx.chakra.button_group(
                    # Rabbit
                    rx.chakra.button(
                        rx.chakra.image(
                            src = "/icons/rabbit_white.png",
                            object_fit = "cover",
                            width = "90%",
                            height = "90%"
                        ),
                        size = "md",
                        variant = "ghost",
                        _hover={
                            "border": "2px solid white",
                            "bg": "transparent",
                        },
                        _active={
                            "border": "2px solid white",
                            "bg": "transparent",
                        }
                    ),
                    # Carrot
                    rx.chakra.button(
                        rx.chakra.image(
                            src = "/icons/carrot_white.png",
                            object_fit = "cover",
                            width = "90%",
                            height = "90%"
                        ),
                        size = "md",
                        variant = "ghost",
                        _hover={
                            "border": "2px solid white",
                            "bg": "transparent",
                        },
                        _active={
                            "border": "2px solid white",
                            "bg": "transparent",
                        }
                    )
                ),
                rx.button("Submit", type="submit"),
            )
        ),

        size = '4',
        justify_content = 'center',
        style={
            "backgroundColor": "rgba(0, 0, 0, 0.8)",
            "color": "white",
        },
        variant = "ghost",
        align_items = "center"
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

def register():
    return bkgrndContainer(registrationForm())