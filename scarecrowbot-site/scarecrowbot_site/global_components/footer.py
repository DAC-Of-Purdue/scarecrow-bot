import reflex as rx

def globalFooter() -> rx.Component:
    return rx.flex(
        rx.link(
            rx.image(src = "/FSEE_branding.jpg", height = "5vh"),
            href = "https://centers.purdue.edu/fusion-studio/"
        ),
        justify_content = "start",
        width = "100%",
        background_color = "white",
        height = "5vh",
        align = "center"
    )
