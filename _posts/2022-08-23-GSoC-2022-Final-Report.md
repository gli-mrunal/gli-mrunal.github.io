---
title: GSoC 2022 Final Report 
author: Mrunal Gavali
date: 2022-08-23 
categories: [GSoC Blogging]
tags: [NLP]
---

## Google Summer of Code 2022: Final Report
In this post, I will share my GSoC’22 journey with the INCF organization on the project `Automatic reviewer matching using Natural Language Processing: Infrastructure for NBDT Journal ` mentored by [Dr. Konrad Kording](https://www.linkedin.com/in/konrad-kording-7284044/), [Dr.Titipat Achakulvisut](https://www.linkedin.com/in/titipata/), [Dr.Daniele Marinazzo](https://scholar.google.com/citations?user=OJbWSLoAAAAJ&hl=en).


It was my first time participating in GSoC. I didn’t interact much on forums before. During GSoC, I contacted many people, raised issues, did pull requests, did code reviews and contributed to other open source projects. Now I have a better understanding of how software is developed in open source and how I can do my part in building more. I want to thank everyone who helped me in my open source journey. 


## Highlights

- [x]  Applied SciBERT to computational Neuroscience data.
- [x] Created BERT embeddings for abstracts.
- [x] Applied Cosine similarity semantic search for recommending reviewers in jupyter notebook.
- [x] Applied KNN using FAISS for semantic similarity search for recommending reviewers in jupyter notebook.
- [x] Developed Login and Authentication for Frontend web application using Typescript with Nextjs and Tailwindcss.
- [x] Using Docker for AI code development and runnning jupyter notebooks.
- [x] Building a Data Loaders for Computational Neuroscience data



## Important Links
1. [The Project description for GSoC](https://summerofcode.withgoogle.com/programs/2022/projects/MDgaQFlg)
2. [GSoC'22 github Code repository](https://github.com/nbdt-journal/automatic-reviewer-assignment) 
3. [PR #14](https://github.com/nbdt-journal/automatic-reviewer-assignment/pull/14) SciBERT Cosine Similarity
4. [PR #8](https://github.com/nbdt-journal/automatic-reviewer-assignment/pull/8) Client --> login implemented for email authentication using firebase
5. Issue: [Pubmed XML data E-utility and automation in retreival](https://github.com/nbdt-journal/automatic-reviewer-assignment/blob/parser_xml_to_csv/scripts/medline_parser/Medline_E-utility.ipynb) & [Data Loader](https://github.com/nbdt-journal/automatic-reviewer-assignment/tree/parser_module/data_loader)


### [GSoC'22 github Code repository](https://github.com/nbdt-journal/automatic-reviewer-assignment) 

## In Progresss / TODO
- [ ] Dockerizing the entire application.
- [ ] Scripting the jupyter notebooks.
- [ ] Integrating the AI application in the web application using Fastapi
- [ ] Deploy web app

## Usage of the Code developed

### Running Data Loader

We have created a Python library for retrieving scientific data from arXiv, bioRxiv, and MEDLINE using their respective APIs, and converting to a it format usable by the automatic reviewer recommendation algorithm.

Link: https://github.com/nbdt-journal/automatic-reviewer-assignment/tree/parser_module/data_loader

Usage
To run the script, simply run:
```sh
  python3 main.py
```

To change the parameters of the function, you can simply change the values in config.yaml.The usage is self-described within the config file. If you wish to not use a certain data source for the parsing, set the parse value to false in the respective parser section.

### Running Web app

 Usage: 

git clone the client branch of the repo and install the dependencies:
```bash
npx install nbdt_frontend with-tailwindcss with-tailwindcss-app
```

### Similarity Search Using KNN with Faiss
`Faiss` is a library developed by [Facebook AI Research](https://ai.facebook.com/). According to their [wikipage](https://github.com/facebookresearch/faiss/wiki),

`Faiss is a library for efficient similarity search and clustering of dense vectors. It contains algorithms that search in sets of vectors of any size, up to ones that possibly do not fit in RAM`

**Here are the steps to build the search engine using the previously built embeddings**:

1. create the flat index: This is used to flat the vectors. The index uses the L2 (Euclidean) distance metrics to mesure the similarity betweeen the query vector and all the vectors (embeddings).
2. add all the vectors to the index
3. define the number K of similar document we want
4. run the similarity search


![knn_1](\Images\GSoC_img\knn_1.png)

![knn_2](\Images\GSoC_img\knn_2.png)

![knn_3](\Images\GSoC_img\knn_3.png)

![knn_4](\Images\GSoC_img\knn_4.png)

**Observation**

- The lower the distance is, the most similar the article is to the query.
- The first document has L2 = 0, which means 100% similarity. This is obvious, because the query was compared with itself.
- We can simply remove it to the analysis.

Note: Refer to [my GSoC blog](https://gli-mrunal.github.io/posts/Docker/) for docker setup for AI.


**GoodBye!**
