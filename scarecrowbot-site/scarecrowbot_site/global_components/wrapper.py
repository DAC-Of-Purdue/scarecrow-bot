import reflex as rx
from scarecrowbot_site.global_components import footer
from scarecrowbot_site.pages import home

# Web app wrapper
def secondaryContainer(*components) -> rx.Component:
    return rx.container(
        rx.vstack(
            *components,
            width = "100%",
            align="center", 
            spacing = "8"
        ),
        size = '4',
        justify_content = 'center',
        center_content = True,
        height = '90vh',
        width = '80vw',
    )

### Main container wrapper (has background image).
def bkgrndContainer(*components) -> rx.Component:
    return rx.flex(
        *components,
        width = "100%",
        height = "95vh",
        background = "url('/bkgrnd.png')",
        background_size = "cover",
        background_position = "right",
        justify_content = "center",
        on_mount = [home.GameState.getStatus, home.LeaderboardState.fetchTeamData]
    )

# Wrapper
def globalWrapper(*components) -> rx.Component:
    return rx.vstack(
        bkgrndContainer(
            secondaryContainer(
                *components
            ),
        ),
    footer.globalFooter()
    )