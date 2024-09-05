import mysql.connector

with mysql.connector.connect(host="localhost", user="rabbitadmin", password="24rRatzRo0l", database="scarecrowbot") as database:
    with database.cursor() as cursor:
        cursor.execute("SELECT * FROM TEAMS ORDER BY HighestScore DESC")
        data = cursor.fetchall()
print(data)