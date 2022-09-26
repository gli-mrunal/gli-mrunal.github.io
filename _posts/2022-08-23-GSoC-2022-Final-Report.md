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


Now change the above code into modular functions for KNN similarity search

1. Perform query
```sh


def process_query(query_text):
    """
    # Create a vector for given query and adjust it for cosine similarity search
    """

    query_vect = convert_single_abstract_to_embedding(sciBERT_tokenizer, model, query_text)
    query_vect = np.array(query_vect)
    query_vect = query_vect.reshape(1, -1)
    return query_vect
```
3. KNN similarity search 

```sh
def get_top_N_articles_knn(query_text, data, K=10):
    """
    Retrieve K=10 articles similar to the query (smaller the L2 distance means more similarity and vice versa)
    """
    query_vect = process_query(query_text)
    revevant_cols = ["title", "abstract",  "doi", "authors", "source", "cos_sim"]
    
    # create the flat index to flat the vectors.
    # The index uses the L2 (Euclidean) distance metrics to mesure the similarity betweeen 
    # the query vector and all the vectors (embeddings).
    embedding_dimension = len(embeddings[0])
    indexFlatL2 = faiss.IndexFlatL2(embedding_dimension)  #768 in this case
    
    # Convert the embeddings list of vectors into a 2D array.
    vectors = np.stack(embeddings)
    indexFlatL2.add(vectors) # add all the vectors to the index 
    
    # Run the similarity query search
    D, I = indexFlatL2.search(query_vector, K)
    

    for i in range(I.shape[1]):
    
        article_index = I[0, i]
    
        abstract = data.iloc[article_index].abstract
        print("** Article #{} **".format(article_index))
        print("** --> Abstract : \n{}**".format(abstract))
    
        authors = data.iloc[article_index].authors
        print("** Authors: \n{}**".format(authors))
    
        doi = data.iloc[article_index].doi
        print("** DOI: \n{}**".format(doi))
    
        source = data.iloc[article_index].source
        print("** Source: \n{} **".format(source))
    
    
    
        print("**---\n L2 Distance: %.2f \n ---**" % D[0, i])
        print("\n\n")
        print("--------------------------------------------------------------------------------------------------------------------\n")
```

---
Now, test on a abstract from data:

```sh
query_text_test = data.iloc[0].abstract # query abstract input

top_articles = get_top_N_articles_knn(query_text_test, data) # 10 similar recommendations in descending order
```
---

OR 


4. User input for customm query

```sh


query_text_test = str(input())
#query_text_test = data.iloc[0].abstract # query abstract input
print("--------------------------------------------------------------------------------------------------------------------\n")
print("********************     RECOMMENDATIONS     *************\n")

top_articles= get_top_N_articles_knn(query_text, data) # apply knn similarity search function

# the recommendations are not for user input abstract

```

**user input**:

`Cognition allows sensory experiences to inform later actions in flexible ways. A new study shows that, in a cognitively demanding task, monkeys store visual information in short-term memory and replay it when they need it to make a decision.`

**Recommendations**:
```sh
** Article #0 **
** --> Abstract : 
To analyse neuron data at scale, neuroscientists expend substantial effort reading documentation, installing dependencies and moving between analysis and visualisation environments. To facilitate this, we have developed a suite of interoperable open-source R packages called the natverse. The natverse allows users to read local and remote data, perform popular analyses including visualisation, clustering and graph-theoretic analysis of neuronal branching. Unlike most tools, the natverse enables comparison of morphology and connectivity across many neurons after imaging or co-registration within a common template space. The natverse also enables transformations between different template spaces and imaging modalities. We demonstrate tools that integrate the vast majority of Drosophila neuroanatomical light microscopy and electron microscopy connectomic datasets. The natverse is an easy-to-use environment for neuroscientists to solve complex, large-scale analysis challenges as well as an open platform to create new code and packages to share with the community.**
** Authors: 
[{'author': 'Bates, A. S.', 'number on Paper': 1, 'institution': 'Division of Neurobiology, MRC Laboratory of Molecular Biology, Cambridge, CB2?0QH, UK'}, {'author': ' Manton, J. D.', 'number on Paper': 2, 'institution': 'Division of Neurobiology, MRC Laboratory of Molecular Biology, Cambridge, CB2?0QH, UK'}, {'author': ' Jagannathan, S. R.', 'number on Paper': 3, 'institution': 'Division of Neurobiology, MRC Laboratory of Molecular Biology, Cambridge, CB2?0QH, UK'}, {'author': ' Costa, M.', 'number on Paper': 4, 'institution': 'Division of Neurobiology, MRC Laboratory of Molecular Biology, Cambridge, CB2?0QH, UK'}, {'author': ' Schlegel, P.', 'number on Paper': 5, 'institution': 'Division of Neurobiology, MRC Laboratory of Molecular Biology, Cambridge, CB2?0QH, UK'}, {'author': ' Rohlfing, T.', 'number on Paper': 6, 'institution': 'Division of Neurobiology, MRC Laboratory of Molecular Biology, Cambridge, CB2?0QH, UK'}, {'author': ' Jefferis, G. S. X. E.', 'number on Paper': 7, 'institution': 'Division of Neurobiology, MRC Laboratory of Molecular Biology, Cambridge, CB2?0QH, UK'}]**
** DOI: 
10.1101/006353**
** Source: 
bioarxiv **
**---
 L2 Distance: 0.00 
 ---**



--------------------------------------------------------------------------------------------------------------------

** Article #779 **
** --> Abstract : 
Computational models are powerful tools for investigating brain function in health and disease. However, biologically detailed neuronal and circuit models are complex and implemented in a range of specialized languages, making them inaccessible and opaque to many neuroscientists. This has limited critical evaluation of models by the scientific community and impeded their refinement and widespread adoption. To address this, we have combined advances in standardizing models, open source software development and web technologies to develop Open Source Brain, a platform for visualizing, simulating, disseminating and collaboratively developing standardized models of neurons and circuits from a range of brain regions. Model structure and parameters can be visualized and their dynamical properties explored through browser-controlled simulations, without writing code. Open Source Brain makes neural models transparent and accessible and facilitates testing, critical evaluation and refinement, thereby helping to improve the accuracy and reproducibility of models, and their dissemination to the wider community.**
** Authors: 
[{'author': 'Gleeson, P.', 'number on Paper': 1, 'institution': 'University College London'}, {'author': ' Cantarelli, M.', 'number on Paper': 2, 'institution': 'University College London'}, {'author': ' Marin, B.', 'number on Paper': 3, 'institution': 'University College London'}, {'author': ' Quintana, A.', 'number on Paper': 4, 'institution': 'University College London'}, {'author': ' Earnshaw, M.', 'number on Paper': 5, 'institution': 'University College London'}, {'author': ' Piasini, E.', 'number on Paper': 6, 'institution': 'University College London'}, {'author': ' Birgiolas, J.', 'number on Paper': 7, 'institution': 'University College London'}, {'author': ' Cannon, R. C.', 'number on Paper': 8, 'institution': 'University College London'}, {'author': ' Cayco-Gajic, N. A.', 'number on Paper': 9, 'institution': 'University College London'}, {'author': ' Crook, S.', 'number on Paper': 10, 'institution': 'University College London'}, {'author': ' Davison, A. P.', 'number on Paper': 11, 'institution': 'University College London'}, {'author': ' Dura-Bernal, S.', 'number on Paper': 12, 'institution': 'University College London'}, {'author': ' Ecker, A.', 'number on Paper': 13, 'institution': 'University College London'}, {'author': ' Hines, M. L.', 'number on Paper': 14, 'institution': 'University College London'}, {'author': ' Idili, G.', 'number on Paper': 15, 'institution': 'University College London'}, {'author': ' Larson, S.', 'number on Paper': 16, 'institution': 'University College London'}, {'author': ' Lytton, W. W.', 'number on Paper': 17, 'institution': 'University College London'}, {'author': ' Majumdar, A.', 'number on Paper': 18, 'institution': 'University College London'}, {'author': ' McDougal, R. A.', 'number on Paper': 19, 'institution': 'University College London'}, {'author': ' Sivagnanam, S.', 'number on Paper': 20, 'institution': 'University College London'}, {'author': ' Solinas, S.', 'number on Paper': 21, 'institution': 'University College London'}, {'author': ' Stanislovas, R.', 'number on Paper': 22, 'institution': 'University College London'}, {'author': ' van Albada, S. J.', 'number on Paper': 23, 'institution': 'University College London'}, {'author': ' Van Geit, W.', 'number on Paper': 24, 'institution': 'University College London'}, {'author': ' Silver, R. A.', 'number on Paper': 25, 'institution': 'University College London'}]**
** DOI: 
10.1101/229484**
** Source: 
bioarxiv **
**---
 L2 Distance: 105.82 
 ---**



--------------------------------------------------------------------------------------------------------------------

** Article #282 **
** --> Abstract : 
Human neuroscience research faces several challenges with regards to reproducibility. While scientists are generally aware that data sharing is an important component of reproducible research, it is not always clear how to usefully share data in a manner that allows other labs to understand and reproduce published findings. Here we describe a new open source tool, AFQ-Browser, that builds an interactive website as a companion to a published diffusion MRI study. Because AFQ-browser is portable -- it runs in any modern web-browser -- it can facilitate transparency and data sharing. Moreover, by leveraging new web-visualization technologies to create linked views between different dimensions of a diffusion MRI dataset (anatomy, quantitative diffusion metrics, subject metadata), AFQ-Browser facilitates exploratory data analysis, fueling new scientific discoveries based on previously published datasets. In an era where Big Data is playing an increasingly prominent role in scientific discovery, so will browser-based tools for exploring high-dimensional datasets, communicating scientific discoveries, sharing and aggregating data across labs, and publishing data alongside manuscripts.**
** Authors: 
[{'author': 'Yeatman, J. D.', 'number on Paper': 1, 'institution': 'The University of Washington'}, {'author': ' Richie-Halford, A.', 'number on Paper': 2, 'institution': 'The University of Washington'}, {'author': ' Smith, J. K.', 'number on Paper': 3, 'institution': 'The University of Washington'}, {'author': ' Keshavan, A.', 'number on Paper': 4, 'institution': 'The University of Washington'}, {'author': ' Rokem, A.', 'number on Paper': 5, 'institution': 'The University of Washington'}]**
** DOI: 
10.1101/182402**
** Source: 
bioarxiv **
**---
 L2 Distance: 108.02 
 ---**



--------------------------------------------------------------------------------------------------------------------

** Article #281 **
** --> Abstract : 
Human neuroscience research faces several challenges with regards to reproducibility. While scientists are generally aware that data sharing is an important component of reproducible research, it is not always clear how to usefully share data in a manner that allows other labs to understand and reproduce published findings. Here we describe a new open source tool, AFQ-Browser, that builds an interactive website as a companion to a published diffusion MRI study. Because AFQ-browser is portable -- it runs in any modern web-browser -- it can facilitate transparency and data sharing. Moreover, by leveraging new web-visualization technologies to create linked views between different dimensions of a diffusion MRI dataset (anatomy, quantitative diffusion metrics, subject metadata), AFQ-Browser facilitates exploratory data analysis, fueling new scientific discoveries based on previously published datasets. In an era where Big Data is playing an increasingly prominent role in scientific discovery, so will browser-based tools for exploring high-dimensional datasets, communicating scientific discoveries, sharing and aggregating data across labs, and publishing data alongside manuscripts.**
** Authors: 
[{'author': 'Yeatman, J. D.', 'number on Paper': 1, 'institution': 'The University of Washington'}, {'author': ' Richie-Halford, A.', 'number on Paper': 2, 'institution': 'The University of Washington'}, {'author': ' Smith, J. K.', 'number on Paper': 3, 'institution': 'The University of Washington'}, {'author': ' Keshavan, A.', 'number on Paper': 4, 'institution': 'The University of Washington'}, {'author': ' Rokem, A.', 'number on Paper': 5, 'institution': 'The University of Washington'}]**
** DOI: 
10.1101/182402**
** Source: 
bioarxiv **
**---
 L2 Distance: 108.02 
 ---**



--------------------------------------------------------------------------------------------------------------------

** Article #3933 **
** --> Abstract : 
Modern neuroscience research often requires the coordination of multiple processes such as stimulus generation, real-time experimental control, as well as behavioral and neural measurements. The technical demands required to simultaneously manage these processes with high temporal fidelity limits the number of labs capable of performing such work. Here we present an open-source network-based parallel processing framework that eliminates these barriers. The Real-Time Experimental Control with Graphical User Interface (REC-GUI) framework offers multiple advantages: (i) a modular design agnostic to coding language(s) and operating system(s) that maximizes experimental flexibility and minimizes researcher effort, (ii) simple interfacing to connect measurement and recording devices, (iii) high temporal fidelity by dividing task demands across CPUs, and (iv) real-time control using a fully customizable and intuitive GUI. Testing results demonstrate that the REC-GUI framework facilitates technically demanding, behavior-contingent neuroscience research. Sample code and hardware configurations are downloadable, and future developments will be regularly released.**
** Authors: 
[{'author': 'Kim, B.', 'number on Paper': 1, 'institution': 'University of Wisconsin - Madison'}, {'author': ' Kenchappa, S. C.', 'number on Paper': 2, 'institution': 'University of Wisconsin - Madison'}, {'author': ' Sunkara, A.', 'number on Paper': 3, 'institution': 'University of Wisconsin - Madison'}, {'author': ' Chang, T.-Y.', 'number on Paper': 4, 'institution': 'University of Wisconsin - Madison'}, {'author': ' Thompson, L.', 'number on Paper': 5, 'institution': 'University of Wisconsin - Madison'}, {'author': ' Doudlah, R.', 'number on Paper': 6, 'institution': 'University of Wisconsin - Madison'}, {'author': ' Rosenberg, A.', 'number on Paper': 7, 'institution': 'University of Wisconsin - Madison'}]**
** DOI: 
10.1101/392654**
** Source: 
bioarxiv **
**---
 L2 Distance: 110.22 
 ---**



--------------------------------------------------------------------------------------------------------------------

** Article #2440 **
** --> Abstract : 
Hyperspectral imaging is a widely used technology for industrial and scientific purposes, but the high cost and large size of commercial setups have made them impractical for most basic research. Here, we designed and implemented a fully open source and low-cost hyperspectral scanner based on a commercial spectrometer coupled to custom optical, mechanical and electronic components. We demonstrate our scanners utility for natural imaging in both terrestrial and underwater environments. Our design provides sub-nm spectral resolution between 350-1000 nm, including the UV part of the light spectrum which has been mostly absent from commercial solutions and previous natural imaging studies. By comparing the full light spectra from natural scenes to the spectral sensitivity of animals, we show how our system can be used to identify subtle variations in chromatic details detectable by different species. In addition, we have created an open access database for hyperspectral datasets collected from natural scenes in the UK and India. Together with comprehensive online build- and use-instructions, our setup provides an inexpensive and customisable solution to gather and share hyperspectral imaging data.**
** Authors: 
[{'author': 'Nevala, N. E.', 'number on Paper': 1, 'institution': 'University of Sussex'}, {'author': ' Baden, T.', 'number on Paper': 2, 'institution': 'University of Sussex'}]**
** DOI: 
10.1101/322172**
** Source: 
bioarxiv **
**---
 L2 Distance: 110.76 
 ---**



--------------------------------------------------------------------------------------------------------------------

** Article #1801 **
** --> Abstract : 
Operant conditioning is a crucial tool in neuroscience research for probing brain function. While molecular, anatomical and even physiological techniques have seen radical increases in throughput, efficiency, and reproducibility in recent years, behavioural tools have seen much less of an improvement. Here we present a fully automated, high-throughput system for self-initiated conditioning of up to 25 group-housed, radio-frequency identification (RFID) tagged mice over periods of several months and >10^6 trials. We validate this \"AutonoMouse\" system in a series of olfactory behavioural tasks and show that acquired data is comparable to previous semi-manual approaches. Furthermore, we use AutonoMouse to systematically probe the impact of graded olfactory bulb lesions on olfactory behaviour and resolve the long-standing conundrum about the apparent lack of impact of lesions on olfactory abilities. The modular nature and open-source design of AutonoMouse should allow for similar robust and systematic assessments across neuroscience research areas.**
** Authors: 
[{'author': 'Erskine, A.', 'number on Paper': 1, 'institution': 'The Francis Crick Institute'}, {'author': ' Bus, T.', 'number on Paper': 2, 'institution': 'The Francis Crick Institute'}, {'author': ' Herb, J. T.', 'number on Paper': 3, 'institution': 'The Francis Crick Institute'}, {'author': ' Schaefer, A. T.', 'number on Paper': 4, 'institution': 'The Francis Crick Institute'}]**
** DOI: 
10.1101/291815**
** Source: 
bioarxiv **
**---
 L2 Distance: 117.73 
 ---**



--------------------------------------------------------------------------------------------------------------------

** Article #1622 **
** --> Abstract : 
Alzheimers disease (AD) is a progressive neurodegenerative disorder that currently affects 36 million people worldwide with no effective treatment available. Development of AD follows a distinctive pattern in the brain and is poorly modelled in animals. Therefore, it is vital to widen both the spatial scope of the study of AD and prioritise the study of human brains. Here we show that functionally distinct human brain regions show varying and region-specific changes in protein expression. These changes provide novel insights into the progression of disease, novel AD-related pathways, the presence of a  gradient of protein expression change from less to more affected regions, and the presence of a  protective protein expression profile in the cerebellum. This spatial proteomics analysis provides a framework which can underpin current research and opens new avenues of interest to enhance our understanding of molecular pathophysiology of AD, provides new targets for intervention and broadens the conceptual frameworks for future AD research.**
** Authors: 
[{'author': 'Xu, J.', 'number on Paper': 1, 'institution': 'The University of Manchester'}, {'author': ' Patassini, S.', 'number on Paper': 2, 'institution': 'The University of Manchester'}, {'author': ' Rustogi, N.', 'number on Paper': 3, 'institution': 'The University of Manchester'}, {'author': ' Riba-Garcia, I.', 'number on Paper': 4, 'institution': 'The University of Manchester'}, {'author': ' Hale, B. D.', 'number on Paper': 5, 'institution': 'The University of Manchester'}, {'author': ' Phillips, A. M.', 'number on Paper': 6, 'institution': 'The University of Manchester'}, {'author': ' Waldvogel, H.', 'number on Paper': 7, 'institution': 'The University of Manchester'}, {'author': ' Haines, R.', 'number on Paper': 8, 'institution': 'The University of Manchester'}, {'author': ' Bradbury, P.', 'number on Paper': 9, 'institution': 'The University of Manchester'}, {'author': ' Stevens, A.', 'number on Paper': 10, 'institution': 'The University of Manchester'}, {'author': ' Faull, R. L.', 'number on Paper': 11, 'institution': 'The University of Manchester'}, {'author': ' Dowsey, A. W.', 'number on Paper': 12, 'institution': 'The University of Manchester'}, {'author': ' Cooper, G. J.', 'number on Paper': 13, 'institution': 'The University of Manchester'}, {'author': ' Unwin, R.', 'number on Paper': 14, 'institution': 'The University of Manchester'}]**
** DOI: 
10.1101/283705**
** Source: 
bioarxiv **
**---
 L2 Distance: 119.27 
 ---**



--------------------------------------------------------------------------------------------------------------------

** Article #1203 **
** --> Abstract : 
It is widely assumed that cells must be physically isolated to study their molecular profiles. However, intact tissue samples naturally exhibit variation in cellular composition, which drives covariation of cell-class-specific molecular features. By analyzing transcriptional covariation in 7221 intact CNS samples from 840 individuals representing billions of cells, we reveal the core transcriptional identities of major CNS cell classes in humans. By modeling intact CNS transcriptomes as a function of variation in cellular composition, we identify cell-class-specific transcriptional differences in Alzheimers disease, among brain regions, and between species. Among these, we show that PMP2 is expressed by human but not mouse astrocytes and significantly increases mouse astrocyte size upon ectopic expression in vivo, causing them to more closely resemble their human counterparts. Our work is available as an online resource (http://oldhamlab.ctec.ucsf.edu) and provides a generalizable strategy for determining the core molecular features of cellular identity in intact biological systems.**
** Authors: 
[{'author': 'Kelley, K. W.', 'number on Paper': 1, 'institution': 'University of California, San Francisco'}, {'author': ' Inoue, H.', 'number on Paper': 2, 'institution': 'University of California, San Francisco'}, {'author': ' Molofsky, A. V.', 'number on Paper': 3, 'institution': 'University of California, San Francisco'}, {'author': ' Oldham, M. C.', 'number on Paper': 4, 'institution': 'University of California, San Francisco'}]**
** DOI: 
10.1101/265397**
** Source: 
bioarxiv **
**---
 L2 Distance: 125.81 
 ---**



--------------------------------------------------------------------------------------------------------------------

** Article #1131 **
** --> Abstract : 
The interpretation of neuronal spike train recordings often relies on abstract statistical models that allow for principled parameter estimation and model selection but provide only limited insights into underlying microcircuits. In contrast, mechanistic models are useful to interpret microcircuit dynamics, but are rarely quantitatively matched to experimental data due to methodological challenges. Here we present analytical methods to efficiently fit spiking circuit models to single-trial spike trains. Using derived likelihood functions, we statistically infer the mean and variance of hidden inputs, neuronal adaptation properties and connectivity for coupled integrate-and-fire neurons. Comprehensive evaluations on synthetic data, validations using ground truth in-vitro and in-vivo recordings, and comparisons with existing techniques demonstrate that parameter estimation is very accurate and efficient, even for highly subsampled networks. Our methods bridge statistical, data-driven and theoretical, model-based neurosciences at the level of spiking circuits, for the purpose of a quantitative, mechanistic interpretation of recorded neuronal population activity.**
** Authors: 
[{'author': 'Ladenbauer, J.', 'number on Paper': 1, 'institution': 'Technische Universität Berlin'}, {'author': ' Ostojic, S.', 'number on Paper': 2, 'institution': 'Technische Universität Berlin'}]**
** DOI: 
10.1101/261016**
** Source: 
bioarxiv **
**---
 L2 Distance: 130.74 
 ---**



--------------------------------------------------------------------------------------------------------------------
```


**Observation**

- The lower the distance is, the most similar the article is to the query.
- The first document has L2 = 0, which means 100% similarity. This is obvious, because the query was compared with itself.
- We can simply remove it to the analysis.

Note: Refer to [my GSoC blog](https://gli-mrunal.github.io/posts/Docker/) for docker setup for AI.


**GoodBye!**
