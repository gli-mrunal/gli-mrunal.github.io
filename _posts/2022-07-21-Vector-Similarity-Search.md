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

## Cosine Similarity Search


## References
[1.] [Semantic Search](https://www.pinecone.io/learn/semantic-search/)
[2.] []()
