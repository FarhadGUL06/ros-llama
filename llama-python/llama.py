import json
import ast
from llama_cpp import Llama

# Load action and object vocab
with open("./json/en/actions.json") as f:
    action_map = json.load(f)
    valid_actions = list(action_map.keys())

with open("./json/en/objects.json") as f:
    object_map = json.load(f)
    valid_objects = list(object_map.values())

# Initialize the model
llm = Llama(
    model_path="./models/llama3.2/unsloth/Llama-3.2-3B-Instruct-Q8_0.gguf",
    n_ctx=1024,
    n_threads=16,
)

# Build prompt for converting input to command
def make_command_prompt(user_input):
    return f"""<|start_header_id|>system<|end_header_id|>
You are a command interpreter that converts natural language into structured action commands.

Respond with a flat Python-style list like:
['action1', 'object1', 'action2', 'object2', ...]
or ['action1', 'object1', 'object2', 'action2', 'object3']

🧠 Use the following valid actions:
{valid_actions}

📦 Use the following valid object names:
{valid_objects}

If the input includes multiple steps, break them down in order.
Write only the command list, no additional text, no emojis, no explanations.
Give only the array, no other text, no errors or invalid objects.
Keep in mind that there are sentences that may use "it" or "them" to refer to previous objects, use the context to resolve these and put the correct object names in the list.
Input: {user_input}
Output:
<|eot_id|><|start_header_id|>assistant<|end_header_id|>
"""

# Main loop
print("🤖 Command Interpreter with LLaMA - Type 'exit' to quit")

while True:
    user_input = input("\nYou: ")
    if user_input.lower() in ["exit", "quit"]:
        print("Goodbye!")
        break

    # Prompt construction
    command_prompt = make_command_prompt(user_input)

    print("🧠 LLaMA:", end=" ", flush=True)
    assistant_response = ""
    for chunk in llm(
        command_prompt,
        max_tokens=200,
        stop=["<|eot_id|>"],
        temperature=0.2,
        top_p=0.95,
        repeat_penalty=1.1,
        stream=True
    ):
        token = chunk["choices"][0]["text"]
        print(token, end="", flush=True)
        assistant_response += token

    print()
    #print(assistant_response.strip())

'''
    # Parse the model's response
    try:
        parsed = ast.literal_eval(assistant_response.strip())
        if not isinstance(parsed, list) or len(parsed) % 2 != 0:
            raise ValueError("Expected a flat list of alternating actions and objects.")

        #print("✅ Structured Command:", parsed)
        print(parsed)
        # Validate contents
        for i in range(0, len(parsed), 2):
            action = parsed[i]
            obj = parsed[i + 1]

    except Exception as e:
        print("❌ Failed to parse response:", e)
        print("Raw output:", assistant_response.strip())
'''