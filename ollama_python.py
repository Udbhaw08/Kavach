import requests
import json

OLLAMA_URL = "http://localhost:11434/api/chat"

def ask_ollama(messages, model="llama3.2"):
    """Send messages to Ollama and get clean response (non-streaming)."""
    payload = {
        "model": model,
        "messages": messages,
        "stream": False
    }
    
    res = requests.post(OLLAMA_URL, json=payload)
    data = res.json()
    return data["message"]["content"]

def main():
    print("\n=== Local ChatGPT (Powered by Ollama) ===")
    print("Type 'exit' to quit.\n")

    messages = []  # Chat history

    while True:
        user_input = input("You: ")

        if user_input.lower() in ["exit", "quit"]:
            print("Goodbye!")
            break

        # Add user message
        messages.append({"role": "user", "content": user_input})

        # Get reply
        reply = ask_ollama(messages)

        # Add assistant reply to history
        messages.append({"role": "assistant", "content": reply})

        print("\nAI:", reply, "\n")

if __name__ == "__main__":
    main()
