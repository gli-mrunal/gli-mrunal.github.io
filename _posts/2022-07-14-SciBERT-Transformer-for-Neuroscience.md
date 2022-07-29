---
title: SciBERT Transformer for Neuroscience
author: Mrunal
date: 2022-07-14 
categories: [GSoC Blogging]
tags: [GSoC]
math: true
mermaid: true

---

## Hugging Face Transformer model for Neuroscience JSON data

The objective of the project is to apply pretrained SciBERT transformer model and Cosine Similarity for recommending reviewers who have published neuroscience research papers on semantically similar research topic as the user's input abstract query.

**Overall Approach**
1. Load the pretrained SciBert model and tokenizer
2. Vectorize documents by creating embeddings
3. Semantic Similarity search by Cosine Similarity

For the purpose of creating embeddings, the bioRxiv Neuroscience data is used. 

## Step 1 : Import Libraries:

```python
import pandas as pd
import numpy as np
from tqdm import tqdm
import warnings

# Hugging Face Transformer libraries
!pip install transformers
import torch
from transformers import BertTokenizer,  AutoModelForSequenceClassification

# Similarity search: cosine similarity search 
from sklearn.metrics.pairwise import cosine_similarity

warnings.filterwarnings("ignore")
```

## Step 2: load the bioRxiv Neuroscience data in JSON format 

```python
data = pd.read_json("bioarxiv_parsed.json") 
print("Data Shape: {}".format(data.shape))

```

## Step 3: Load Pretrained SciBERT model and tokenizer

The SciBERT model is used for creating embeddings for the abstracts in the Neuroscience research papers. 
Note that in the code snippet below the `output_hidden_states` is set to `True` so that we can extract the embeddings.
```python
# Get the SciBERT pretrained model path from Allen AI repo
pretrained_model = 'allenai/scibert_scivocab_uncased'

# Get the tokenizer from the previous path
sciBERT_tokenizer = BertTokenizer.from_pretrained(pretrained_model, 
                                          do_lower_case=True)

# Get the model
model = AutoModelForSequenceClassification.from_pretrained(pretrained_model,
                                                          output_attentions=False,
                                                          output_hidden_states=True)
```

> model.eval()
gives the architecture of the model as seen below:

```python
BertForSequenceClassification(
  (bert): BertModel(
    (embeddings): BertEmbeddings(
      (word_embeddings): Embedding(31090, 768, padding_idx=0)
      (position_embeddings): Embedding(512, 768)
      (token_type_embeddings): Embedding(2, 768)
      (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
      (dropout): Dropout(p=0.1, inplace=False)
    )
    (encoder): BertEncoder(
      (layer): ModuleList(
        (0): BertLayer(
          (attention): BertAttention(
            (self): BertSelfAttention(
              (query): Linear(in_features=768, out_features=768, bias=True)
              (key): Linear(in_features=768, out_features=768, bias=True)
              (value): Linear(in_features=768, out_features=768, bias=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
            (output): BertSelfOutput(
              (dense): Linear(in_features=768, out_features=768, bias=True)
              (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
          )
          (intermediate): BertIntermediate(
            (dense): Linear(in_features=768, out_features=3072, bias=True)
            (intermediate_act_fn): GELUActivation()
          )
          (output): BertOutput(
            (dense): Linear(in_features=3072, out_features=768, bias=True)
            (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
            (dropout): Dropout(p=0.1, inplace=False)
          )
        )
        (1): BertLayer(
          (attention): BertAttention(
            (self): BertSelfAttention(
              (query): Linear(in_features=768, out_features=768, bias=True)
              (key): Linear(in_features=768, out_features=768, bias=True)
              (value): Linear(in_features=768, out_features=768, bias=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
            (output): BertSelfOutput(
              (dense): Linear(in_features=768, out_features=768, bias=True)
              (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
          )
          (intermediate): BertIntermediate(
            (dense): Linear(in_features=768, out_features=3072, bias=True)
            (intermediate_act_fn): GELUActivation()
          )
          (output): BertOutput(
            (dense): Linear(in_features=3072, out_features=768, bias=True)
            (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
            (dropout): Dropout(p=0.1, inplace=False)
          )
        )
        (2): BertLayer(
          (attention): BertAttention(
            (self): BertSelfAttention(
              (query): Linear(in_features=768, out_features=768, bias=True)
              (key): Linear(in_features=768, out_features=768, bias=True)
              (value): Linear(in_features=768, out_features=768, bias=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
            (output): BertSelfOutput(
              (dense): Linear(in_features=768, out_features=768, bias=True)
              (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
          )
          (intermediate): BertIntermediate(
            (dense): Linear(in_features=768, out_features=3072, bias=True)
            (intermediate_act_fn): GELUActivation()
          )
          (output): BertOutput(
            (dense): Linear(in_features=3072, out_features=768, bias=True)
            (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
            (dropout): Dropout(p=0.1, inplace=False)
          )
        )
        (3): BertLayer(
          (attention): BertAttention(
            (self): BertSelfAttention(
              (query): Linear(in_features=768, out_features=768, bias=True)
              (key): Linear(in_features=768, out_features=768, bias=True)
              (value): Linear(in_features=768, out_features=768, bias=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
            (output): BertSelfOutput(
              (dense): Linear(in_features=768, out_features=768, bias=True)
              (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
          )
          (intermediate): BertIntermediate(
            (dense): Linear(in_features=768, out_features=3072, bias=True)
            (intermediate_act_fn): GELUActivation()
          )
          (output): BertOutput(
            (dense): Linear(in_features=3072, out_features=768, bias=True)
            (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
            (dropout): Dropout(p=0.1, inplace=False)
          )
        )
        (4): BertLayer(
          (attention): BertAttention(
            (self): BertSelfAttention(
              (query): Linear(in_features=768, out_features=768, bias=True)
              (key): Linear(in_features=768, out_features=768, bias=True)
              (value): Linear(in_features=768, out_features=768, bias=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
            (output): BertSelfOutput(
              (dense): Linear(in_features=768, out_features=768, bias=True)
              (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
          )
          (intermediate): BertIntermediate(
            (dense): Linear(in_features=768, out_features=3072, bias=True)
            (intermediate_act_fn): GELUActivation()
          )
          (output): BertOutput(
            (dense): Linear(in_features=3072, out_features=768, bias=True)
            (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
            (dropout): Dropout(p=0.1, inplace=False)
          )
        )
        (5): BertLayer(
          (attention): BertAttention(
            (self): BertSelfAttention(
              (query): Linear(in_features=768, out_features=768, bias=True)
              (key): Linear(in_features=768, out_features=768, bias=True)
              (value): Linear(in_features=768, out_features=768, bias=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
            (output): BertSelfOutput(
              (dense): Linear(in_features=768, out_features=768, bias=True)
              (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
          )
          (intermediate): BertIntermediate(
            (dense): Linear(in_features=768, out_features=3072, bias=True)
            (intermediate_act_fn): GELUActivation()
          )
          (output): BertOutput(
            (dense): Linear(in_features=3072, out_features=768, bias=True)
            (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
            (dropout): Dropout(p=0.1, inplace=False)
          )
        )
        (6): BertLayer(
          (attention): BertAttention(
            (self): BertSelfAttention(
              (query): Linear(in_features=768, out_features=768, bias=True)
              (key): Linear(in_features=768, out_features=768, bias=True)
              (value): Linear(in_features=768, out_features=768, bias=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
            (output): BertSelfOutput(
              (dense): Linear(in_features=768, out_features=768, bias=True)
              (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
          )
          (intermediate): BertIntermediate(
            (dense): Linear(in_features=768, out_features=3072, bias=True)
            (intermediate_act_fn): GELUActivation()
          )
          (output): BertOutput(
            (dense): Linear(in_features=3072, out_features=768, bias=True)
            (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
            (dropout): Dropout(p=0.1, inplace=False)
          )
        )
        (7): BertLayer(
          (attention): BertAttention(
            (self): BertSelfAttention(
              (query): Linear(in_features=768, out_features=768, bias=True)
              (key): Linear(in_features=768, out_features=768, bias=True)
              (value): Linear(in_features=768, out_features=768, bias=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
            (output): BertSelfOutput(
              (dense): Linear(in_features=768, out_features=768, bias=True)
              (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
          )
          (intermediate): BertIntermediate(
            (dense): Linear(in_features=768, out_features=3072, bias=True)
            (intermediate_act_fn): GELUActivation()
          )
          (output): BertOutput(
            (dense): Linear(in_features=3072, out_features=768, bias=True)
            (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
            (dropout): Dropout(p=0.1, inplace=False)
          )
        )
        (8): BertLayer(
          (attention): BertAttention(
            (self): BertSelfAttention(
              (query): Linear(in_features=768, out_features=768, bias=True)
              (key): Linear(in_features=768, out_features=768, bias=True)
              (value): Linear(in_features=768, out_features=768, bias=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
            (output): BertSelfOutput(
              (dense): Linear(in_features=768, out_features=768, bias=True)
              (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
          )
          (intermediate): BertIntermediate(
            (dense): Linear(in_features=768, out_features=3072, bias=True)
            (intermediate_act_fn): GELUActivation()
          )
          (output): BertOutput(
            (dense): Linear(in_features=3072, out_features=768, bias=True)
            (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
            (dropout): Dropout(p=0.1, inplace=False)
          )
        )
        (9): BertLayer(
          (attention): BertAttention(
            (self): BertSelfAttention(
              (query): Linear(in_features=768, out_features=768, bias=True)
              (key): Linear(in_features=768, out_features=768, bias=True)
              (value): Linear(in_features=768, out_features=768, bias=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
            (output): BertSelfOutput(
              (dense): Linear(in_features=768, out_features=768, bias=True)
              (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
          )
          (intermediate): BertIntermediate(
            (dense): Linear(in_features=768, out_features=3072, bias=True)
            (intermediate_act_fn): GELUActivation()
          )
          (output): BertOutput(
            (dense): Linear(in_features=3072, out_features=768, bias=True)
            (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
            (dropout): Dropout(p=0.1, inplace=False)
          )
        )
        (10): BertLayer(
          (attention): BertAttention(
            (self): BertSelfAttention(
              (query): Linear(in_features=768, out_features=768, bias=True)
              (key): Linear(in_features=768, out_features=768, bias=True)
              (value): Linear(in_features=768, out_features=768, bias=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
            (output): BertSelfOutput(
              (dense): Linear(in_features=768, out_features=768, bias=True)
              (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
          )
          (intermediate): BertIntermediate(
            (dense): Linear(in_features=768, out_features=3072, bias=True)
            (intermediate_act_fn): GELUActivation()
          )
          (output): BertOutput(
            (dense): Linear(in_features=3072, out_features=768, bias=True)
            (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
            (dropout): Dropout(p=0.1, inplace=False)
          )
        )
        (11): BertLayer(
          (attention): BertAttention(
            (self): BertSelfAttention(
              (query): Linear(in_features=768, out_features=768, bias=True)
              (key): Linear(in_features=768, out_features=768, bias=True)
              (value): Linear(in_features=768, out_features=768, bias=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
            (output): BertSelfOutput(
              (dense): Linear(in_features=768, out_features=768, bias=True)
              (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
              (dropout): Dropout(p=0.1, inplace=False)
            )
          )
          (intermediate): BertIntermediate(
            (dense): Linear(in_features=768, out_features=3072, bias=True)
            (intermediate_act_fn): GELUActivation()
          )
          (output): BertOutput(
            (dense): Linear(in_features=3072, out_features=768, bias=True)
            (LayerNorm): LayerNorm((768,), eps=1e-12, elementwise_affine=True)
            (dropout): Dropout(p=0.1, inplace=False)
          )
        )
      )
    )
    (pooler): BertPooler(
      (dense): Linear(in_features=768, out_features=768, bias=True)
      (activation): Tanh()
    )
  )
  (dropout): Dropout(p=0.1, inplace=False)
  (classifier): Linear(in_features=768, out_features=2, bias=True)
)

```

## Step 4: Create an embedding for a given text data using SciBERT pre-trained model

This function `convert_single_abstract_to_embedding` is mostly inspired by the BERT Word Embeddings Tutorial of [Chris McCormick](https://mccormickml.com/2019/05/14/BERT-word-embeddings-tutorial/#3-extracting-embeddings) and [Zoumana Keita](https://towardsdatascience.com/scientific-documents-similarity-search-with-deep-learning-using-transformers-scibert-d47c4e501590). It aims to create an embedding for a given text data using a pre-trained model.

```python
def convert_single_abstract_to_embedding(tokenizer, model, in_text, MAX_LEN = 510):
    
    input_ids = tokenizer.encode(
                        in_text, 
                        add_special_tokens = True, 
                        max_length = MAX_LEN,                           
                   )    

    results = pad_sequences([input_ids], maxlen=MAX_LEN, dtype="long", 
                              truncating="post", padding="post")
    
    # Remove the outer list.
    input_ids = results[0]

    # Create attention masks    
    attention_mask = [int(i>0) for i in input_ids]
    
    # Convert to tensors.
    input_ids = torch.tensor(input_ids)
    attention_mask = torch.tensor(attention_mask)

    # Add an extra dimension for the "batch" (even though there is only one 
    # input in this batch.)
    input_ids = input_ids.unsqueeze(0)
    attention_mask = attention_mask.unsqueeze(0)
    
    # Put the model in "evaluation" mode, meaning feed-forward operation.
    model.eval()

 
    # Run the text through BERT, and collect all of the hidden states produced
    # from all 12 layers. 
    with torch.no_grad():        
        logits, encoded_layers = model(
                                    input_ids = input_ids, 
                                    token_type_ids = None, 
                                    attention_mask = attention_mask,
                                    return_dict=False)

    layer_i = 12 # The last BERT layer before the classifier.
    batch_i = 0 # Only one input in the batch.
    token_i = 0 # The first token, corresponding to [CLS]
        
    # Extract the embedding.
    embedding = encoded_layers[layer_i][batch_i][token_i]

    # Move to the CPU and convert to numpy ndarray.
    embedding = embedding.detach().cpu().numpy()

    return(embedding)
    
 ```
    
    
Now we can use the model and tokenizer to generate an embedding for the 3rd input_abstract as a way of testing as seen in the code snippet below:
    
```python
    from keras_preprocessing.sequence import pad_sequences

    input_abstract = data.abstract.iloc[3]

    abstract_embedding = convert_single_abstract_to_embedding(sciBERT_tokenizer, model, input_abstract)

    print('Embedding shape: {}'.format(abstract_embedding.shape))
    
```
    
 Note to run the above code snippet make sure you have installed keras and tensorflow. You can install both of them in the jupyter notebook in the following way:
    
```sh
    !pip install keras
    
    !pip3 install tensorflow
```
    
The output of the 3rd input abstract embedding shape is:
    
```
    Embedding shape: (768,)
    Embedding is composed of 768 values.
```
   
## Step 5: Create Embedding for all the abstracts
   
```python
   def convert_all_abstract_text_to_embedding(df):
    
    # The list of all the embeddings
    embeddings = []
    
    # Get overall text data
    overall_text_data = data.abstract.values
    
    # Loop over all the comment and get the embeddings
    for abstract in tqdm(overall_text_data):
        
        # Get the embedding 
        embedding = convert_single_abstract_to_embedding(sciBERT_tokenizer, model, abstract)
        
        #add it to the list
        embeddings.append(embedding)
        
    print("Conversion Done!")
    
    return embeddings
```
   
Note that creating embeddings for all the abstracts in the 3948 BioRxiv Neuroscience research papers takes atleast 2 hours in aws sagemaker.
   
```python
   # This task can take a lot of time depending on the sample_size value 
   embeddings = convert_all_abstract_text_to_embedding(data)
```
   
## Step 6: Save the embeddings for future use
   
```python
   
   embeddings = np.array(embeddings)
   np.save('embeddings.npy', embeddings)
```
   
## Step 7: Load the saved .npy embeddings
   
```pyhton
   embeddings = np.load('embeddings.npy')
```
## Step 8: Create a new column that will contain embedding of each body text

```python
   
def create_final_embeddings(df, embeddings):
    
    df["embeddings"] = embeddings
    df["embeddings"] = df["embeddings"].apply(lambda emb: np.array(emb))
    df["embeddings"] = df["embeddings"].apply(lambda emb: emb.reshape(1, -1))
    
    return df
    
```
    
 To see the output:
    
  ```pyhton
    data = create_final_embeddings(data, embeddings)
    data.head(3)
 ```
 
![embeddigns column](\Images\GSoC_img\emb.png)
    
## References
1. @inproceedings{beltagy-etal-2019-scibert, title = "SciBERT: A Pretrained Language Model for Scientific Text", author = "Beltagy, Iz and Lo, Kyle and Cohan, Arman", booktitle = "EMNLP", year = "2019", publisher = "Association for Computational Linguistics", url = "https://www.aclweb.org/anthology/D19-1371" }

2. @article{johnson2019billion, title={Billion-scale similarity search with {GPUs}}, author={Johnson, Jeff and Douze, Matthijs and J{\'e}gou, Herv{\'e}}, journal={IEEE Transactions on Big Data}, volume={7}, number={3}, pages={535--547}, year={2019}, publisher={IEEE} }

3. “Bert Word Embeddings Tutorial.” BERT Word Embeddings Tutorial · Chris McCormick, 14 May 2019, https://mccormickml.com/2019/05/14/BERT-word-embeddings-tutorial/#3-extracting-embeddings.

4. Keita, Zoumana. “Scientific Documents Similarity Search with Deep Learning Using Transformers (Scibert).” Medium, Towards Data Science, 17 Jan. 2022, https://towardsdatascience.com/scientific-documents-similarity-search-with-deep-learning-using-transformers-scibert-d47c4e501590.

5. @article{Beltagy2020Longformer, title={Longformer: The Long-Document Transformer}, author={Iz Beltagy and Matthew E. Peters and Arman Cohan}, journal={arXiv:2004.05150}, year={2020}, }
    


