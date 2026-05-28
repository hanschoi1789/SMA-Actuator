import pandas as pd

file_path = 'Exo_Bending_StaticOptimization_force .sto'

# 1. OpenSim '.sto' 파일 구조상 데이터가 시작되는 지점('endheader') 찾기
with open(file_path, 'r') as f:
    for i, line in enumerate(f):
        if 'endheader' in line:
            header_idx = i
            break

# 2. 데이터프레임으로 불러오기 (탭 분리 기준)
df = pd.read_csv(file_path, skiprows=header_idx + 1, sep='\t')

# 3. 시간(Time)을 각도(Angle)로 변환 (2초 동안 60도 회전 -> 초당 30도)
df['Angle'] = df['time'] * 30.0

# 4. 각도와 양쪽 액츄에이터 장력 데이터만 추출
# 파일 내의 'Exo_Right'와 'Exo_Left'가 액츄에이터 데이터를 의미합니다.
result_df = df[['Angle', 'Exo_Right', 'Exo_Left']]

# 5. 새로운 CSV 파일로 저장
output_path = 'Exo_Actuator_Tension_by_Angle.csv'
result_df.to_csv(output_path, index=False)

print("데이터 변환 완료. 미리보기:")
print(result_df.head())