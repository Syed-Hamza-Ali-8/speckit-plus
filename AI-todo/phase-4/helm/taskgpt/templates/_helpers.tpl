{{/*
Expand the name of the chart.
*/}}
{{- define "taskgpt.name" -}}
{{- default .Chart.Name .Values.nameOverride | trunc 63 | trimSuffix "-" }}
{{- end }}

{{/*
Create a default fully qualified app name.
*/}}
{{- define "taskgpt.fullname" -}}
{{- if .Values.fullnameOverride }}
{{- .Values.fullnameOverride | trunc 63 | trimSuffix "-" }}
{{- else }}
{{- $name := default .Chart.Name .Values.nameOverride }}
{{- if contains $name .Release.Name }}
{{- .Release.Name | trunc 63 | trimSuffix "-" }}
{{- else }}
{{- printf "%s-%s" .Release.Name $name | trunc 63 | trimSuffix "-" }}
{{- end }}
{{- end }}
{{- end }}

{{/*
Create chart name and version as used by the chart label.
*/}}
{{- define "taskgpt.chart" -}}
{{- printf "%s-%s" .Chart.Name .Chart.Version | replace "+" "_" | trunc 63 | trimSuffix "-" }}
{{- end }}

{{/*
Common labels
*/}}
{{- define "taskgpt.labels" -}}
helm.sh/chart: {{ include "taskgpt.chart" . }}
{{ include "taskgpt.selectorLabels" . }}
{{- if .Chart.AppVersion }}
app.kubernetes.io/version: {{ .Chart.AppVersion | quote }}
{{- end }}
app.kubernetes.io/managed-by: {{ .Release.Service }}
{{- end }}

{{/*
Selector labels
*/}}
{{- define "taskgpt.selectorLabels" -}}
app.kubernetes.io/name: {{ include "taskgpt.name" . }}
app.kubernetes.io/instance: {{ .Release.Name }}
{{- end }}

{{/*
Backend labels
*/}}
{{- define "taskgpt.backendLabels" -}}
{{ include "taskgpt.labels" . }}
app: {{ .Values.backend.name }}
tier: backend
{{- end }}

{{/*
Backend selector labels
*/}}
{{- define "taskgpt.backendSelectorLabels" -}}
app: {{ .Values.backend.name }}
{{- end }}

{{/*
Frontend labels
*/}}
{{- define "taskgpt.frontendLabels" -}}
{{ include "taskgpt.labels" . }}
app: {{ .Values.frontend.name }}
tier: frontend
{{- end }}

{{/*
Frontend selector labels
*/}}
{{- define "taskgpt.frontendSelectorLabels" -}}
app: {{ .Values.frontend.name }}
{{- end }}
