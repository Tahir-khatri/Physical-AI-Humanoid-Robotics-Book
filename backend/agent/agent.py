import os
import logging
from typing import List, Dict, Any
import cohere
from qdrant_client import QdrantClient
import openai # Added import

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Constants for Qdrant and Cohere
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "physical-ai-book-v1")
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY") # Added
COHERE_EMBED_MODEL = "embed-english-v3.0" # Cohere embedding model used by ingestion pipeline

if QDRANT_URL is None or QDRANT_API_KEY is None or COHERE_API_KEY is None or OPENAI_API_KEY is None:
    logging.error("Missing QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY, or OPENAI_API_KEY environment variables.")
    raise ValueError("Missing API configuration.")

# Initialize Cohere client
co = cohere.Client(COHERE_API_KEY)

# Initialize Qdrant client
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Initialize OpenAI client
client = openai.OpenAI(api_key=OPENAI_API_KEY) # Added

def qdrant_retriever_tool(query: str, top_k: int = 5) -> str:
    """
    A tool function to retrieve relevant document chunks from Qdrant based on a query.
    This function will be called by the OpenAI agent.

    Args:
        query (str): The user's query string.
        top_k (int): The number of top relevant chunks to retrieve.

    Returns:
        str: A formatted string containing the retrieved document chunks.
    """
    logging.info(f"QdrantRetriever tool called with query: '{query}'")

    # Generate embedding for the query using Cohere
    try:
        response = co.embed(
            texts=[query],
            model=COHERE_EMBED_MODEL,
            input_type="search_query"
        )
        query_embedding = response.embeddings[0]
    except Exception as e:
        logging.error(f"Error generating embedding for query: {e}")
        return "Error: Could not generate embedding for the query."

    # Search Qdrant
    try:
        search_result = qdrant_client.search(
            collection_name=QDRANT_COLLECTION_NAME,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True,
        )
    except Exception as e:
        logging.error(f"Error searching Qdrant collection: {e}")
        return "Error: Could not retrieve information from the knowledge base."

    if not search_result:
        return "No relevant documents found in the knowledge base."

    # Format results for the agent
    formatted_results = []
    for i, hit in enumerate(search_result):
        payload = hit.payload
        formatted_results.append(f"Document {i+1} (Source: {payload.get('url', 'N/A')}, Section: {payload.get('section_title', 'N/A')}):\n{payload.get('text', 'N/A')}\n")

    return "\n".join(formatted_results)

def run_agent_turn(user_question: str, chat_history: List[Dict[str, str]] = None, context: Optional[str] = None) -> str:
    """
    Initializes and runs the OpenAI Assistant with a user's question, optional chat history,
    and optional contextual text.

    Args:
        user_question (str): The question from the user.
        chat_history (List[Dict[str, str]]): A list of previous messages in the format
                                              [{"role": "user", "content": "..."}, {"role": "assistant", "content": "..."}]
        context (Optional[str]): Optional contextual text to prepend to the user's question.

    Returns:
        str: The agent's answer.
    """
    logging.info(f"Running agent turn for question: '{user_question[:50]}...' (Context: {bool(context)})")
    if chat_history is None:
        chat_history = []

    full_user_message = user_question
    if context:
        full_user_message = f"Based on this additional context: '{context}', answer the question: '{user_question}'"

    try:
        # Define the Assistant's tools
        tools = [
            {
                "type": "function",
                "function": {
                    "name": "qdrant_retriever_tool",
                    "description": "Retrieves relevant document chunks from Qdrant based on a search query.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "The search query."},
                            "top_k": {"type": "integer", "description": "The number of top results to retrieve."},
                        },
                        "required": ["query"],
                    },
                },
            }
        ]

        # Initialize the Assistant
        assistant = client.beta.assistants.create(
            name="RAG Book Assistant",
            instructions="You are a helpful assistant that answers questions based on provided book content. Use the qdrant_retriever_tool to find relevant information from the book before answering. If you cannot find relevant information, state that you cannot answer from the provided knowledge.",
            model="gpt-4-turbo-preview", # Or another suitable model
            tools=tools,
        )

        # Create a thread
        thread = client.beta.threads.create()

        # Add chat history to the thread
        for message in chat_history:
            client.beta.threads.messages.create(
                thread_id=thread.id,
                role=message["role"],
                content=message["content"],
            )

        # Add the user's current message
        client.beta.threads.messages.create(
            thread_id=thread.id,
            role="user",
            content=full_user_message,
        )

        # Run the Assistant
        run = client.beta.threads.runs.create(
            thread_id=thread.id,
            assistant_id=assistant.id,
        )

        # Wait for the run to complete
        while run.status == "queued" or run.status == "in_progress" or run.status == "requires_action":
            time.sleep(1) # Wait for 1 second
            run = client.beta.threads.runs.retrieve(thread_id=thread.id, run_id=run.id)

            if run.status == "requires_action":
                tool_outputs = []
                for tool_call in run.required_action.submit_tool_outputs.tool_calls:
                    if tool_call.function.name == "qdrant_retriever_tool":
                        args = eval(tool_call.function.arguments) # CAUTION: eval can be dangerous with untrusted input
                        output = qdrant_retriever_tool(query=args.get("query"), top_k=args.get("top_k", 5))
                        tool_outputs.append({
                            "tool_call_id": tool_call.id,
                            "output": output,
                        })
                client.beta.threads.runs.submit_tool_outputs(
                    thread_id=thread.id,
                    run_id=run.id,
                    tool_outputs=tool_outputs,
                )
                run = client.beta.threads.runs.retrieve(thread_id=thread.id, run_id=run.id) # Re-retrieve run status after submitting tool outputs

        # Retrieve messages and return the assistant's response
        messages = client.beta.threads.messages.list(thread_id=thread.id)
        for message in messages.data:
            if message.role == "assistant":
                for content_block in message.content:
                    if content_block.type == "text":
                        return content_block.text.value
        return "No response from assistant."

    except Exception as e:
        logging.error(f"Error running agent turn: {e}")
        return "An error occurred while processing your request."
