from google import genai

client = genai.Client(api_key="AIzaSyAGM6vwEUdqj9AeIWA7N0qTyuMJuy9VJRU")

try:
    for m in client.models.list():
        print(m.name)
except Exception as e:
    print(e)
