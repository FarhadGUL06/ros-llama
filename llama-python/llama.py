import json
import ast
from llama_cpp import Llama

input_language = "en"  # Romanian

# Models
LLAMA3_2_UNSLOTH_Q4 = "./models/llama3.2/unsloth/Llama-3.2-3B-Instruct-Q4_K_M.gguf"
LLAMA3_2_UNSLOTH_Q8 = "./models/llama3.2/unsloth/Llama-3.2-3B-Instruct-Q8_0.gguf"
GEMMA3_UNSLOTH_Q2 = "./models/gemma3/unsloth/gemma-3-12b-it-UD-IQ2_M.gguf"
GEMMA3_UNSLOTH_Q4 = "./models/gemma3/unsloth/gemma-3-12b-it-Q4_K_M.gguf"

MODEL_USED = LLAMA3_2_UNSLOTH_Q4

# Load action and object vocab

if input_language == "en":
    action_file = "./json/en/actions.json"
    object_file = "./json/en/objects.json"
elif input_language == "ro":
    action_file = "./json/ro/actiuni.json"
    object_file = "./json/ro/obiecte.json"


# Load action and object vocab
with open("./json/en/actions.json") as f:
    action_map = json.load(f)
    valid_actions = list(action_map.keys())

with open("./json/en/objects.json") as f:
    object_map = json.load(f)
    valid_objects = list(object_map.values())

# Initialize the model
llm = Llama(
    model_path=MODEL_USED,
    n_ctx=1024,
    n_threads=16,
)

# Build prompt for converting input to command


RO_PROMPT = f"""
Sunteti un interpret de comenzi care converteste limbajul natural in comenzi de actiune structurate.

Raspundeti cu o lista plata in stil Python precum:
['action1', 'object1', 'action2', 'object2', ...]
sau ['action1', 'object1', 'object2', 'action2', 'object3']

🧠 Utilizati urmatoarele actiuni valide:
{valid_actions}

📦 Utilizati urmatoarele nume de obiecte valide:
{valid_objects}

Daca intrarea include mai multe etape, defalcati-le in ordine.
Scrieti numai lista de comenzi, niciun text suplimentar, niciun emoji, nicio explicatie.
Dati numai matricea, niciun alt text, nicio eroare sau obiect invalid.
Tineti cont de faptul ca exista propozitii care pot folosi "it" sau "them" pentru a se referi la obiectele anterioare, folositi contextul pentru a le rezolva si puneti numele corecte ale obiectelor in lista.
Nu includeti in lista cuvinte cu prepozitie precum "cu", "la", "pentru", "inapoi", "in fata" etc.
Pentru comenzile care implica mai multe obiecte, utilizati numele corecte ale obiectelor din lista de mai sus. Exemplu: "pune mingea pe masa" ar trebui sa fie ["pune", "minge", "masa"].

"""


EN_PROMPT = f"""
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
Dont include preposition words like "with", "to", "for", "back", "front" etc. in the list.
For commands that involve multiple objects, use the correct object names from the list above. Exemple: "put the ball on the table" should be ['put', 'ball', 'table'].
"""



def llama_make_command_prompt(user_input):
    if input_language == "ro":
        return f"""<|start_header_id|>system<|end_header_id|>{RO_PROMPT}
Input: {user_input}
Output:
<|eot_id|><|start_header_id|>assistant<|end_header_id|>
"""
    else:
        return f"""<|start_header_id|>system<|end_header_id|>{EN_PROMPT}
Input: {user_input}
Output:
<|eot_id|><|start_header_id|>assistant<|end_header_id|>
"""

def gemma_make_command_prompt(user_input):
    if input_language == "ro":
        return f"""{RO_PROMPT}
<|im_start|>user
{user_input}
<|im_end|>
<|im_start|>assistant
"""
    else:
        return f"""{EN_PROMPT}
<|im_start|>user
{user_input}
<|im_end|>
<|im_start|>assistant
"""

# Main loop
print("🤖 Command Interpreter with LLaMA - Type 'exit' to quit")

while True:
    user_input = input("\nYou: ")
    if user_input.lower() in ["exit", "quit"]:
        print("Goodbye!")
        break

    # Prompt construction
    if MODEL_USED is LLAMA3_2_UNSLOTH_Q4 or MODEL_USED is LLAMA3_2_UNSLOTH_Q8:
        command_prompt = llama_make_command_prompt(user_input)
    elif MODEL_USED is GEMMA3_UNSLOTH_Q2 or MODEL_USED is GEMMA3_UNSLOTH_Q4:
        command_prompt = gemma_make_command_prompt(user_input)

    if MODEL_USED is LLAMA3_2_UNSLOTH_Q4 or MODEL_USED is LLAMA3_2_UNSLOTH_Q8:
        print("🧠 Llama 3.2: ", end=" ", flush=True)
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
    elif MODEL_USED is GEMMA3_UNSLOTH_Q2 or MODEL_USED is GEMMA3_UNSLOTH_Q4:
        print("🧠 Gemma 3: ", end=" ", flush=True)
        assistant_response = ""
        for chunk in llm(
            command_prompt,
            max_tokens=200,
            stop=["<|im_end|>"],
            temperature=0.2,
            top_p=0.95,
            repeat_penalty=1.1,
            stream=True
        ):
            token = chunk["choices"][0]["text"]
            print(token, end="", flush=True)
            assistant_response += token
        
    print()