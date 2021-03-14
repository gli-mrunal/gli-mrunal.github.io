---

title: Portfolio
icon: fas fa-info
order: 4
---



<div class="card-columns">
  {% for project in site.data.portfolio %}
    {% include portfolio/project_card.html %}
  {% endfor %}
</div>

