# check_model.py
import joblib

obj = joblib.load('xgboost_model.pkl')
print("Type:", type(obj))
print("Object:", obj)

if hasattr(obj, 'classes_'):
    print("Classes:", obj.classes_)
if hasattr(obj, 'n_features_in_'):
    print("n_features_in_:", obj.n_features_in_)
if hasattr(obj, 'feature_names_in_'):
    print("Feature names:", obj.feature_names_in_)
