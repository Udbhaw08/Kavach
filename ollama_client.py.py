from ollama import chat
import json

def get_temperature(city: str) -> str:
    temperatures = {
        "New York": "22°C",
        "London": "15°C",
        "Tokyo": "18°C",
    }
    return temperatures.get(city, "Unknown")


messages = [
    {"role": "system",
     "content": """
You are a function-calling model. 
When answering, follow these rules:

1. If you need to call a tool, respond ONLY in this JSON format:

{
  "tool": "get_temperature",
  "arguments": {
    "city": "New York"
  }
}

2. DO NOT output any text outside the JSON.
3. DO NOT explain the JSON.
4. DO NOT add comments.
5. DO NOT include backticks.

If no tool is needed, reply normally in plain text.
"""},

    {"role": "user", "content": "What's the temperature in New York?"}
]

response = chat(
    model="llama3.2",
    messages=messages
)

raw = response["message"]["content"].strip()
print("MODEL RAW OUTPUT:", raw)

# Attempt to parse JSON
tool_call = None
try:
    tool_call = json.loads(raw)
except:
    tool_call = None

if isinstance(tool_call, dict) and "tool" in tool_call:
    tool = tool_call["tool"]
    args = tool_call.get("arguments", {})

    if tool == "get_temperature":
        result = get_temperature(**args)
        print("TOOL RESULT:", result)

else:
    print("FINAL ANSWER:", raw)
