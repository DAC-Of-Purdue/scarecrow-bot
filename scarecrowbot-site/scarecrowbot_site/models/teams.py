from sqlmodel import Field

# For defining the team for auth in the code.
class TeamAuth(rx.model, table = True):
    teamName: str 
    loginPIN: str

# Table for team registration.
class Team(rx.model, table = True)
    teamName: str
    loginPIN: str
    icon: str
    highestScore: int 
    cumulativeScore: int 
    