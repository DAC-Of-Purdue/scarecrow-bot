

import reflex as rx
from scarecrowbot_site.global_components import footer
from scarecrowbot_site.global_components import wrapper

def embedCard() -> rx.Component:
    return rx.card(
        rx.video(
            url = "https://www.youtube.com/embed/MBqH-UfIwrg",
            width = "64vw",
            height = "36vw",
            controls = False,
        ),
        justify_content = "center",
        align_content = "center"
    )



def watch():
    return(
        wrapper.globalWrapper(
            embedCard()
        )
    )