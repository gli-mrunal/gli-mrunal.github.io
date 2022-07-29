---
title: Vector Similarity Search 
author: Mrunal Gavali
date: 2022-07-21 
categories: [GSoC Blogging]
tags: [NLP]
---
## Vector Similarity Search

BERT (Bidirectional Encoder Representations from Transformers) is the most popular deep learning model in natural language processing field.
Through 12 encoder layers, BERT encodes a massive information into a set of dense vectors. There are usually 512 max tokens of these dense vectors for each sentence encoded by BERT model. And each dense vector typically contains 768 values.These dense vectors contain the numerical representations of textual language. Besides, we can also extract these dense vectors — from different layers if we want — but typically vectors are extracted from the final layer.

Then, with 2 correctly encoded dense vectors, a similarity metric like Cosine similarity or L2 distance from KNN can be used to calculate their semantic similarity. Vectors that are more aligned are more semantically alike, and vice versa.

## Query Processing


The similarity analysis is performed between a given abstract query vector and all the embeddings vectors. 

The query processing function is shown code snippet below:

```python
def process_query(query_text):
    """ 
    Create a vector for given query and adjust it for cosine similarity search
    """

    query_vect = convert_single_abstract_to_embedding(sciBERT_tokenizer, model, query_text)
    query_vect = np.array(query_vect)
    query_vect = query_vect.reshape(1, -1)
    return query_vect
```
## Similarity Metric: Cosine Similarity Search
The cosine similarity between two abstracts’ embedding measures how similar those abstracts are, irrespective of the size of those embeddings.
It measures the cosine of the angle between the two vectors projected in a multi-dimensional space as seen in the image below.
- `cosine similarity of 1`:  two abstracts are **100% similar**
- `cosine similarity of 0`: two abstracts have **0% similarity**

![Cosine similarity](\Images\GSoC_img\cosine.png)

The following function returns the top 10 articles similar to the query abstract.

```python
def get_top_N_similar_articles_cosine(query_text, data, top_N=10):
    """
    Retrieve top_N (10 is default value) article abstract similar to the query
    """
    query_vect = process_query(query_text)
    revevant_cols = ["title", "abstract",  "doi", "authors", "source", "cosine_similarity"]
    
    # Run similarity Search
    data["cosine_similarity"] = data["embeddings"].apply(lambda x: cosine_similarity(query_vect, x))
    data["cosine_similarity"] = data["cosine_similarity"].apply(lambda x: x[0][0])
    
    """
    Sort Cosine Similarity Column in Descending Order.
    Below index starts at 1 to remove similarity with itself because it is always 1.
    """
    most_similar_articles = data.sort_values(by='cosine_similarity', ascending=False)[1:top_N+1]
    
    return most_similar_articles[revevant_cols]

```

## Cosine Similarity for any abstract inputted by user

```python
query_text_test = str(input())
print("--------------------------------------------------------------------------------------------------------------------\n")
print("********************     RECOMMENDATIONS     *************\n")

top_articles = get_top_N_similar_articles_cosine(query_text_test, data)  # take input from user and recommend top 10 using cosine similarity
```

For example if the user inputted the following abstract:

```
Our visual environment impacts multiple aspects of cognition including perception, attention and memory, yet most studies traditionally remove or control the external environment. As a result, we have a limited understanding of neurocognitive processes beyond the controlled lab environment. Here, we aim to study neural processes in real-world environments, while also maintaining a degree of control over perception. To achieve this, we combined mobile EEG (mEEG) and augmented reality (AR), which allows us to place virtual objects into the real world. We validated this AR and mEEG approach using a well-characterised cognitive response-the face inversion effect. Participants viewed upright and inverted faces in three EEG tasks (1) a lab-based computer task, (2) walking through an indoor environment while seeing face photographs, and (3) walking through an indoor environment while seeing virtual faces. We find greater low frequency EEG activity for inverted compared to upright faces in all experimental tasks, demonstrating that cognitively relevant signals can be extracted from mEEG and AR paradigms. This was established in both an epoch-based analysis aligned to face events, and a GLM-based approach that incorporates continuous EEG signals and face perception states. Together, this research helps pave the way to exploring neurocognitive processes in real-world environments while maintaining experimental control using AR.
```

Then the output of the top 10 similar authors (reviewers recommendations) associated with the abstracts of the research papers having similar research topic based on cosine similarity metric are:

> top_articles.authors

```
3102    [{'author': 'Kannape, O. A.', 'number on Paper...
906     [{'author': 'Teichmann, L.', 'number on Paper'...
1984    [{'author': 'Nunes, A.', 'number on Paper': 1,...
2343    [{'author': 'Hakim, A.', 'number on Paper': 1,...
2344    [{'author': 'Hakim, A.', 'number on Paper': 1,...
1081    [{'author': 'Etard, O.', 'number on Paper': 1,...
1082    [{'author': 'Etard, O.', 'number on Paper': 1,...
1232    [{'author': 'Sutterer, D. W.', 'number on Pape...
3152    [{'author': 'Betzel, R. F.', 'number on Paper'...
2038    [{'author': 'Javadi, A.-H.', 'number on Paper'...
```

Note: The corresponding abstracts fof these authors can be seen as follows:

> top_articles # top 10 recommendations

![Reviewer Recommendations](\Images\GSoC_img\reviewers_recomm.png)



## References
[1.] [Semantic Search](https://www.pinecone.io/learn/semantic-search/)
[2.] [Scientific Documents Similarity Search With Deep Learning Using Transformers (SciBERT)](https://towardsdatascience.com/scientific-documents-similarity-search-with-deep-learning-using-transformers-scibert-d47c4e501590)
