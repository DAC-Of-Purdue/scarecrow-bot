###### REGISTRATION PAGE ############
# Created for DAC by Autumn Denny.  #
# Functions of page:                #
#   - register new teams            #
#   - customize team color and icon #
#####################################

import reflex as rx
from scarecrowbot_site.global_components import wrapper

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
                        "border": "2px solid tomato",
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
                        "border": "2px solid tomato",
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
                        "border": "2px solid tomato",
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
                        "border": "2px solid tomato",
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
                        "border": "2px solid tomato",
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
                        "border": "2px solid tomato",
                        "bg": "#d211f0",
                    }),
                # black
                rx.chakra.button(
                    size = "sm",
                    bg = "#000000",
                    _hover={
                        "border": "2px solid white",
                        "bg": "#000000",
                    },
                    _active={
                        "border": "2px solid tomato",
                        "bg": "#000000",
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
                        "border": "2px solid tomato",
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
        justify_content = 'center',
        style={
            "backgroundColor": "rgba(255, 255, 255, 1.0)",
        },
        variant = "ghost",
        align_items = "center"
    )

def register():
    return wrapper.globalWrapper(
        registrationForm()
    )