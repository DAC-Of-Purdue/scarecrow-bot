# These are the API calls for the game engine to communicate with our database

import mysql.connector
from fastapi import HTTPException
from fastapi.responses import JSONResponse


# GAME ENGINE PASS CODE. KEEP PRIVATE.
PASSCODE = "0101KurzweiL99"


# Stops the current game.
async def stopGame(passcode: str):
    try:
        if passcode == PASSCODE:
            with mysql.connector.connect(host="localhost", user="rabbitadmin", password="24rRatzRo0l", database="scarecrowbot") as database:
                with database.cursor() as cursor:
                    cursor.execute("UPDATE GAME_STATUS SET Status = False, CurrentTeam=0  where Status = True")
                database.commit()
            return JSONResponse(
                status_code=200,
                content={"message": "OK"}
            )
        else:
            return JSONResponse(
                status_code=401,
                content={"message": "Unauthorized"}
            )
    except:
        return JSONResponse(
            status_code=500,
            content={"message": "Failed"}
        )

async def startGame(passcode: str, loginPIN: str):
    try:
        if passcode == PASSCODE:
            with mysql.connector.connect(host="localhost", user="rabbitadmin", password="24rRatzRo0l", database="scarecrowbot") as database:
                try:    
                    with database.cursor() as cursor:
                        cursor.execute("SELECT * FROM TEAMS WHERE LoginPIN = " + str(loginPIN))
                        data = cursor.fetchall()
                        teamID = data[0][0]
                        query = "UPDATE GAME_STATUS SET Status = True, CurrentTeam=" + str(teamID) +  " where Status = False"
                        cursor.execute(query)
                    database.commit()
                except:
                    return JSONResponse(
                        status_code=404,
                        content={"message": "Not Found"}
                    )   
            return JSONResponse(
                status_code=200,
                content={"message": "OK"}
            )
        else:
            return JSONResponse(
                status_code=401,
                content={"message": "Unauthorized"}
            )
    except:
        return JSONResponse(
            status_code=500,
            content={"message": "Failed"}
        )

async def getStatus():
    gameStatus = 0
    teamID = 0
    try:
        with mysql.connector.connect(host="localhost", user="rabbitadmin", password="24rRatzRo0l", database="scarecrowbot") as database:
            with database.cursor() as cursor:
                cursor.execute("SELECT * FROM GAME_STATUS")
                data = cursor.fetchall()
                gameStatus = str(data[0][0])
                teamID = str(data[0][1])
        return JSONResponse(status_code = 200, content = {"isGameOn": gameStatus, "teamID": teamID})
    except:
        return JSONResponse(
            status_code=500,
            content={"message": "Failed"}
        )



# Pushes the final scores to the database.
async def pushScore(teamid:str, score:int, passcode:str,):
    try:
        if passcode == PASSCODE:
            try:
                with mysql.connector.connect(host="localhost", user="rabbitadmin", password="24rRatzRo0l", database="scarecrowbot") as database:
                    with database.cursor() as cursor:
                        cursor.execute("SELECT * FROM TEAMS WHERE TeamID = " + str(teamid))
                        data = cursor.fetchall()
                        latestScore = data[0][2]
                        highestScore = data[0][3]
                        
                        if score > highestScore:
                            query = "UPDATE TEAMS SET HighestScore = " + str(score) + " WHERE TeamID = " + teamid
                            cursor.execute(query)
                            database.commit()
                            return JSONResponse(
                                status_code=200,
                                content={"message":"OK"}
                            )           
                        else:
                            query = "UPDATE TEAMS SET LatestScore = " + str(score) + " WHERE TeamID = " + teamid
                            cursor.execute(query)
                            database.commit()
                            return JSONResponse(
                                status_code=200,
                                content={"message":"OK"}
                            )                    
                                
                        teamID = str(data[0][1])
            except:
                return JSONResponse(
                    status_code=404,
                    content={"message": "Not Found"}
                )
        else:
            return JSONResponse(
                status_code=401,
                content={"message": "Unauthorized"}
            )
    except:
        return JSONResponse(
            status_code=500,
            content={"message":"Failed"}
        )


