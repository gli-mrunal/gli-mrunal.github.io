---
title: GSoC 2022 Final Report 
author: Mrunal Gavali
date: 2022-08-23 
categories: [GSoC Blogging]
tags: [NLP]
---

## Google Summer of Code 2022: Final Report
In this post, I will share my GSoC’22 journey with the INCF organization on the project `Automatic reviewer matching using Natural Language Processing: Infrastructure for NBDT Journal ` mentored by [Dr. Konrad Kording](https://www.linkedin.com/in/konrad-kording-7284044/), [Dr.Titipat Achakulvisut](https://www.linkedin.com/in/titipata/), [Dr.Daniele Marinazzo](https://scholar.google.com/citations?user=OJbWSLoAAAAJ&hl=en).




### [GSoC'22 github Code repository](https://github.com/nbdt-journal/automatic-reviewer-assignment) 


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
3. PR [GSoC'22 github Code repository](https://github.com/nbdt-journal/automatic-reviewer-assignment) 
4. [PR #14](https://github.com/nbdt-journal/automatic-reviewer-assignment/pull/14) SciBERT Cosine Similarity
5. [PR #8](https://github.com/nbdt-journal/automatic-reviewer-assignment/pull/8) Client --> login implemented for email authentication using firebase
6. Issue: [Pubmed XML data E-utility and automation in retreival](https://github.com/nbdt-journal/automatic-reviewer-assignment/blob/parser_xml_to_csv/scripts/medline_parser/Medline_E-utility.ipynb) & [Data Loader](https://github.com/nbdt-journal/automatic-reviewer-assignment/tree/parser_module/data_loader)

## In Progresss / TODO
- [ ] Dockerizing the entire application.
- [ ] Scripting the jupyter notebooks.
- [ ] Integrating the AI application in the web application using Fastapi


## Usage of the Code developed

# Running Web app

 Usage: Execute [`create-next-app`](https://github.com/vercel/next.js/tree/canary/packages/create-next-app) with [npm](https://docs.npmjs.com/cli/init), [Yarn]  (https://yarnpkg.com/lang/en/docs/cli/create/), or [pnpm](https://pnpm.io) to bootstrap the example:

```bash
npx create-next-app --example with-tailwindcss with-tailwindcss-app
# or
yarn create next-app --example with-tailwindcss with-tailwindcss-app
# or
pnpm create next-app --example with-tailwindcss with-tailwindcss-app
```

Deploy it to the cloud with [Vercel](https://vercel.com/new?utm_source=github&utm_medium=readme&utm_campaign=next-example) ([Documentation](https://nextjs.org/docs/deployment)).

# Running Data Loader

We have created a Python library for retrieving scientific data from arXiv, bioRxiv, and MEDLINE using their respective APIs, and converting to a it format usable by the automatic reviewer recommendation algorithm.

Link: https://github.com/nbdt-journal/automatic-reviewer-assignment/tree/parser_module/data_loader

Usage
To run the script, simply run:
```sh
  python3 main.py
```

To change the parameters of the function, you can simply change the values in config.yaml.The usage is self-described within the config file. If you wish to not use a certain data source for the parsing, set the parse value to false in the respective parser section.

