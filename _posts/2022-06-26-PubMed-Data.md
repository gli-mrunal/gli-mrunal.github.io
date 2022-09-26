---
title: PubMed Data 
author: Mrunal
date: 2022-06-26 
categories: [GSoC Blogging, AI]
tags: [GSoC]
math: true
mermaid: true
---

## Automation for Pubmed XML data retrieval using E-utility


##  Data

Data is colelcted from the Pubmed Medline, Arxiv and Bioarxiv journals for neuroscience papers published for the past 5 years.
The fields collected are:
- `PMID`
- `Title`
- `Abstract`
- `List of Authors`
- `Affiliation`
- `source` (Medline | arxiv | bioarxiv)
- `Publish year`

The data needs to be parsed in a JSON format as shown below:

![JSON data](\Images\GSoC_img\json_data.png)



The PubMed data is in XML or text format. The XML format is chosen for parsing so that there is durability and efficiency while parsing the required fields from the original  XML data into JSON data.

Since, we only need to find computational neuroscience or neuroscience related research papers, The PubMed E-utility is used to retrieve the data through automation.
This blog walks over the process of retrieving the more than 10,000 research papers at once using the PubMed E-utility automation for 'computational+neuroscience' term.

### Running Data Loader

We have created a Python library for retrieving scientific data from arXiv, bioRxiv, and MEDLINE using their respective APIs, and converting to a it format usable by the automatic reviewer recommendation algorithm.

Link: https://github.com/nbdt-journal/automatic-reviewer-assignment/tree/parser_module/data_loader

Usage
To run the script, simply run:
```sh
  python3 main.py
```

To change the parameters of the function, you can simply change the values in config.yaml.The usage is self-described within the config file. If you wish to not use a certain data source for the parsing, set the parse value to false in the respective parser section.


## References
1. [Pubmed]()
2. [Arxiv]()
3. [Bioarxiv]()
4. [text data E-utility](https://erilu.github.io/pubmed-abstract-compiler/)





