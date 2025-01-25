import sys
import pandas as pd
import matplotlib.pyplot as plt

def main(csv_file):
    # CSV 파일 읽기
    try:
        data = pd.read_csv(csv_file, header=None, names=['time', 'x', 'y', 'z', 'ratio'])
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    # 필요한 열 확인
    required_columns = ['time', 'x', 'y', 'z', 'ratio']
    if not all(col in data.columns for col in required_columns):
        print(f"CSV file must contain the following columns: {required_columns}")
        return

    # 시간과 ratio 그래프
    plt.figure()
    plt.plot(data['time'], data['ratio'], label='Ratio', color='blue')
    plt.xlabel('Time')
    plt.ylabel('Ratio')
    plt.title('Time vs Ratio')
    plt.legend()
    plt.grid(True)

    # 시간과 z 그래프
    plt.figure()
    plt.plot(data['time'], data['z'], label='Z', color='red')
    plt.xlabel('Time')
    plt.ylabel('Z')
    plt.title('Time vs Z')
    plt.legend()
    plt.grid(True)

    # 그래프 보여주기
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script_name.py csv_file_name.csv")
    else:
        csv_file = sys.argv[1]
        main(csv_file)

