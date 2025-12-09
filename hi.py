from google import genai

client = genai.Client(api_key="AIzaSyAGM6vwEUdqj9AeIWA7N0qTyuMJuy9VJRU")



response = client.models.generate_content(
    model="models/gemini-2.5-flash",
    contents="Explain ML clearly in simple words.",
    config={
        "temperature": 0.7,
        "max_output_tokens": 8192
    }
)

# Newer SDK versions support .text
# print(response)
print(response.candidates[0].content.parts[0].text)