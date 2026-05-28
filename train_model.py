import pandas as pd
import xgboost as xgb
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import classification_report, accuracy_score
import joblib

def train():
    # 1. 데이터 로드
    try:
        df = pd.read_csv("imu_dataset.csv")
    except FileNotFoundError:
        print("❌ 'imu_dataset.csv' 파일이 없습니다. 데이터를 먼저 수집하세요.")
        return

    print(f"✅ 데이터 로드 성공! 총 {len(df)}개의 샘플이 있습니다.")

    X = df.drop(columns=['label'])
    y = df['label']

    # 🌟 핵심: XGBoost는 문자열을 그대로 쓸 수 없으므로 숫자로 인코딩 (예: standing->2, walking->3)
    le = LabelEncoder()
    y_encoded = le.fit_transform(y)

    # 2. 학습용/테스트용 분리
    X_train, X_test, y_train, y_test = train_test_split(X, y_encoded, test_size=0.2, random_state=42, stratify=y_encoded)

    # 3. XGBoost 모델 생성 및 학습
    print("\n🚀 XGBoost 모델 학습을 시작합니다...")
    model = xgb.XGBClassifier(
        n_estimators=200,      # 나무의 개수 (Random Forest보다 보통 크게 잡음)
        max_depth=6,           # 나무의 깊이
        learning_rate=0.1,     # 학습률 (오답을 고쳐나가는 보폭)
        random_state=42,
        eval_metric='mlogloss'
    )
    model.fit(X_train, y_train)

    # 4. 성능 평가
    predictions = model.predict(X_test)
    acc = accuracy_score(y_test, predictions)
    print(f"\n📊 검증 정확도(Accuracy): {acc * 100:.2f}%\n")
    
    # 평가 리포트는 다시 사람이 보기 편하게 문자열로 출력
    print("상세 리포트:")
    print(classification_report(y_test, predictions, target_names=le.classes_))

    # 5. 모델과 라벨 인코더를 딕셔너리로 묶어서 함께 저장 (추정 파일에서 둘 다 필요함)
    model.save_model("xgboost_model.json")
    joblib.dump(le, "label_encoder.pkl")
    
    print("💾 학습된 모델('xgboost_model.json')과 인코더('label_encoder.pkl')가 저장되었습니다.")
    
if __name__ == '__main__':
    train()