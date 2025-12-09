import json
import ollama
import asyncio

# -----------------------------
# FUNCTIONS (TOOLS)
# -----------------------------

def get_antonyms(word: str) -> str:
    """Get the antonym of a given word."""

    words = {
        "hot": "cold",
        "small": "big",
        "weak": "strong",
        "light": "dark",
        "lighten": "darken",
        "dark": "bright",
    }

    return json.dumps({"antonym": words.get(word, "Not available")})


def get_flight_times(departure: str, arrival: str) -> str:
    """Get flight information"""

    flights = {
        "NYC-LAX": {
            "departure": "08:00 AM",
            "arrival": "11:30 AM",
            "duration": "5h 30m",
        },
        "LAX-NYC": {
            "departure": "02:00 PM",
            "arrival": "10:30 PM",
            "duration": "5h 30m",
        },
        "LHR-JFK": {
            "departure": "10:00 AM",
            "arrival": "01:00 PM",
            "duration": "8h 00m",
        },
        "JFK-LHR": {
            "departure": "09:00 PM",
            "arrival": "09:00 AM",
            "duration": "7h 00m",
        },
    }

    key = f"{departure}-{arrival}".upper()
    return json.dumps(flights.get(key, {"error": "Flight not found"}))


# -----------------------------
# MAIN CHAT FUNCTION
# -----------------------------

async def run(model: str, user_input: str):
    client = ollama.AsyncClient()

    # Strong system instruction to FORCE tool-calling
    messages = [
        {
            "role": "system",
            "content": (
                "You MUST call a function. NEVER answer directly.\n"
                "If the user asks for an antonym or opposite -> call get_antonyms.\n"
                "If the user asks about a flight time -> call get_flight_times.\n"
            )
        },
        {"role": "user", "content": user_input}
    ]

    # First call to model
    response = await client.chat(
        model=model,
        messages=messages,
        tools=[
            {
                "type": "function",
                "function": {
                    "name": "get_antonyms",
                    "description": "Get antonym of a word",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "word": {
                                "type": "string",
                                "description": "The word"
                            }
                        },
                        "required": ["word"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "get_flight_times",
                    "description": "Get flight times between two cities",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "departure": {"type": "string"},
                            "arrival": {"type": "string"}
                        },
                        "required": ["departure", "arrival"]
                    }
                }
            }
        ]
    )

    # Debug (optional)
    print("\nMODEL RAW OUTPUT:", response, "\n")

    # Extract tool call
    tool_calls = response["message"].get("tool_calls")

    if not tool_calls:
        print("❌ MODEL DID NOT CALL ANY FUNCTION")
        print("Model replied:", response["message"]["content"])
        return

    print("🔧 Function call detected!")

    available_functions = {
        "get_antonyms": get_antonyms,
        "get_flight_times": get_flight_times
    }

    # Handle each tool call
    for tool in tool_calls:
        fn_name = tool["function"]["name"]
        args = tool["function"]["arguments"]
        fn = available_functions[fn_name]

        print(f"➡ Calling: {fn_name} with args {args}")

        result = fn(**args)
        print("✔ Function result:", result)


# -----------------------------
# CLI LOOP
# -----------------------------

while True:
    user_input = input("\nAsk me anything => ")

    if user_input.lower() == "exit":
        break

    asyncio.run(run("llama3.2", user_input))
