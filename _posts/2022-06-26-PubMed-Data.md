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



## References
1. [Pubmed]()
2. [Arxiv]()
3. [Bioarxiv]()





