import sqlite3
import os
from ament_index_python.packages import get_package_share_directory

class DBManager():
    def __init__(self, db_name):
        try:
            db_file = os.path.join(get_package_share_directory("makne_db"), "db", db_name)
            self.connection = sqlite3.connect(db_file, check_same_thread=False)
            self.cursor = self.connection.cursor()
        except sqlite3.Error as e:
            print(f"An error occurred while connecting to the database: {e}")
            self.connection = None
            self.cursor = None
            
    def list_tables(self):
        """데이터베이스에 존재하는 모든 테이블의 이름을 반환하는 함수"""
        if not self.connection or not self.cursor:
            print("No database connection.")
            return []

        try:
            self.cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
            tables = self.cursor.fetchall()
            return [table[0] for table in tables]
        except sqlite3.Error as e:
            print(f"An error occurred while listing tables: {e}")
            return []
            
    def get_all_data(self, table_name):
        """해당 테이블의 모든 데이터를 가져오는 함수"""
        if not self.connection or not self.cursor:
            print("No database connection")
            return []
        
        try:
            query = f"SELECT * FROM {table_name}"
            self.cursor.execute(query)
            results = self.cursor.fetchall()
            return results
        except sqlite3.Error as e:
            print(f"An error occurred while fetching data: {e}")
            return []
        

    def get_data_with_condition(self, table_name, condition_column, condition_value):
        """특정 조건을 만족하는 데이터를 가져오는 함수"""
        if not self.connection or not self.cursor:
            print("No database connection.")
            return []

        try:
            query = f"SELECT * FROM {table_name} WHERE {condition_column} = ?"
            self.cursor.execute(query, (condition_value,))
            results = self.cursor.fetchall()
            return results
        except sqlite3.Error as e:
            print(f"An error occurred while fetching data: {e}")
            return []
        
    def get_column_data(self, table_name, column_name):
        """모든 location_name 값을 가져오는 함수"""
        if not self.connection or not self.cursor:
            print("No database connection.")
            return []

        try:
            query = f"SELECT {column_name} FROM {table_name}"
            self.cursor.execute(query)
            results = self.cursor.fetchall()
            return [row[0] for row in results]
        except sqlite3.Error as e:
            print(f"An error occurred while fetching location names: {e}")
            return []

    def insert_data(self, table_name, column_names, values):
        """데이터를 테이블에 삽입하는 함수. 오류가 발생하면 예외처리"""
        if not self.connection or not self.cursor:
            print("No database connection.")
            return

        try:
            columns = ", ".join(column_names)
            placeholders = ", ".join("?" * len(values))
            query = f"INSERT INTO {table_name} ({columns}) VALUES ({placeholders})"
            self.cursor.execute(query, values)
            self.connection.commit()
            print("Data inserted successfully.")
        except sqlite3.Error as e:
            print(f"An error occurred while inserting data: {e}")
            self.connection.rollback()

    def __del__(self):
        if self.connection:
            try:
                self.connection.close()
            except sqlite3.Error as e:
                print(f"An error occurred while closing the database connection: {e}")
        
    

def main():
    db_manager = DBManager("makne_db")

if __name__ == "__main__":
    main()