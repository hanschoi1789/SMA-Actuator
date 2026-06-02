import pandas as pd

def update_dataset():
    filename = "imu_dataset.csv"
    
    # 1. 기존 데이터 로드
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"❌ '{filename}' 파일을 찾을 수 없습니다.")
        return

    # 2. 이미 열이 추가되어 있는지 확인 (중복 방지)
    if 'thigh_mean_diff' in df.columns:
        print("⚠ 이미 'thigh_mean_diff' 열이 존재합니다.")
        return

    # 3. 새로운 특성 계산: |왼쪽 허벅지 평균 - 오른쪽 허벅지 평균|
    df['thigh_mean_diff'] = abs(df['l_thigh_mean'] - df['r_thigh_mean'])

    # 4. 덮어쓰기 저장
    df.to_csv(filename, index=False)
    print(f"✅ 기존 CSV 파일에 'thigh_mean_diff' 열이 성공적으로 추가되었습니다! (총 {len(df)}행)")

if __name__ == '__main__':
    update_dataset()